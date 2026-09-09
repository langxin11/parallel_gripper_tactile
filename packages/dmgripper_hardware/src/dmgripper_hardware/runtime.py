"""只读 DM 电机状态刷新的运行时。"""

from __future__ import annotations

import time
from collections.abc import Callable

from .config import Usb2CanDeviceConfig
from .protocol import (
    USB2CAN_RX_FRAME_SIZE,
    MotorFeedback,
    RegisterReply,
    Usb2CanFrameExtractor,
    Usb2CanProtocol,
)
from .transport import ByteTransport, TransportClosedError


class StateRefreshTimeoutError(TimeoutError):
    """在配置的状态刷新时限内未收到目标电机的有效反馈。"""


class ShortWriteError(OSError):
    """USB2CAN 未接受完整状态刷新请求帧。"""


class RegisterReadTimeoutError(TimeoutError):
    """在配置时限内未收到目标寄存器回复。"""


class DmResponseReceiver:
    """等待并解析一条目标反馈或寄存器回复的共享接收器。

    发送某些达妙控制帧后，参考 ROS 2 驱动会直接接收该帧产生的反馈。此类只消费
    已发送命令之后的串口字节，绝不额外发送状态请求，避免让反馈队列积压。
    """

    def __init__(
        self,
        config: Usb2CanDeviceConfig,
        protocol: Usb2CanProtocol,
        transport: ByteTransport,
        *,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        """保存接收依赖，不产生 I/O。"""
        self._config = config
        self._protocol = protocol
        self._transport = transport
        self._clock = clock
        self._extractor = Usb2CanFrameExtractor()
        self._pending_frames: list[bytes] = []

    def receive_feedback(self) -> MotorFeedback:
        """读取至多一个总超时内的目标电机反馈，不发送查询帧。"""
        result = self._receive_matching(
            lambda frame: self._protocol.parse_feedback(
                frame,
                motor_id=self._config.motor_id,
                master_id=self._config.master_id,
            ),
            StateRefreshTimeoutError("等待目标电机状态反馈超时"),
        )
        assert isinstance(result, MotorFeedback)
        return result

    def receive_register_reply(self, register_id: int) -> RegisterReply:
        """读取目标电机指定寄存器的回复，不发送查询帧。"""
        result = self._receive_matching(
            lambda frame: self._matching_register_reply(frame, register_id),
            RegisterReadTimeoutError("等待目标电机寄存器回复超时"),
        )
        assert isinstance(result, RegisterReply)
        return result

    def _matching_register_reply(self, frame: bytes, register_id: int) -> RegisterReply | None:
        """只接受当前电机和请求寄存器的回复，忽略其他帧。"""
        reply = self._protocol.parse_register_reply(frame, master_id=self._config.master_id)
        if (
            reply is None
            or reply.motor_id != self._config.motor_id
            or reply.register_id != register_id
        ):
            return None
        return reply

    def _receive_matching[T](
        self, parser: Callable[[bytes], T | None], timeout_error: Exception
    ) -> T:
        """在配置总时限内分帧并返回第一个匹配解析结果。"""
        if not self._transport.is_open:
            raise TransportClosedError("状态接收传输尚未打开")
        deadline = self._clock() + self._config.timeout_s
        while True:
            while self._pending_frames:
                frame = self._pending_frames.pop(0)
                parsed = parser(frame)
                if parsed is not None:
                    return parsed
            remaining = deadline - self._clock()
            if remaining <= 0.0:
                raise timeout_error
            received = self._transport.read(USB2CAN_RX_FRAME_SIZE, timeout_s=remaining)
            if not received:
                raise timeout_error
            self._pending_frames.extend(self._extractor.feed(received))


class DmRegisterReader:
    """只读访问达妙寄存器的有界运行时。"""

    def __init__(
        self,
        config: Usb2CanDeviceConfig,
        protocol: Usb2CanProtocol,
        transport: ByteTransport,
        receiver: DmResponseReceiver,
    ) -> None:
        """保存依赖，不打开端口也不发送报文。"""
        self._config = config
        self._protocol = protocol
        self._transport = transport
        self._receiver = receiver

    def read_u32(self, register_id: int) -> int:
        """发送一次只读寄存器请求，并返回匹配的无符号值。"""
        if not self._transport.is_open:
            raise TransportClosedError("寄存器读取传输尚未打开")
        request = self._protocol.make_read_register_packet(self._config.motor_id, register_id)
        written = self._transport.write(request, timeout_s=self._config.timeout_s)
        if written != len(request):
            raise ShortWriteError(
                f"寄存器读取请求短写：期望 {len(request)} 字节，实际 {written} 字节"
            )
        return self._receiver.receive_register_reply(register_id).value


class DmStateRefresher:
    """一次只发送状态刷新请求并读取一帧目标反馈的显式运行时。

    本类不在构造时打开传输。调用方必须先显式 `open`，或使用上下文管理器；状态刷新
    唯一允许的发送内容是 `Usb2CanProtocol.make_feedback_request` 产生的请求帧。它不
    实现使能、失能、置零、控制报文或寄存器读写。

    Args:
        config: 已严格验证的 USB2CAN 连接与目标 ID 配置。
        protocol: 使用调用方显式给定量程的协议编解码器。
        transport: 已注入的字节传输；默认不会创建真实设备。
        clock: 单调时钟，测试可注入以验证超时。
        receiver: 可选的共享接收器，供同一串口上的命令回复连续解析。
    """

    def __init__(
        self,
        config: Usb2CanDeviceConfig,
        protocol: Usb2CanProtocol,
        transport: ByteTransport,
        *,
        clock: Callable[[], float] = time.monotonic,
        receiver: DmResponseReceiver | None = None,
    ) -> None:
        """保存依赖，不打开端口或写入任何帧。"""
        self._config = config
        self._protocol = protocol
        self._transport = transport
        self._clock = clock
        self._receiver = receiver or DmResponseReceiver(config, protocol, transport, clock=clock)

    @property
    def is_open(self) -> bool:
        """报告底层传输是否由调用方显式打开。"""
        return self._transport.is_open

    def open(self) -> None:
        """按配置显式打开传输。"""
        self._transport.open(self._config.port, self._config.baud_rate)

    def close(self) -> None:
        """关闭传输；不发送任何 CAN 帧。"""
        self._transport.close()

    def __enter__(self) -> DmStateRefresher:
        """打开传输并返回只读状态刷新器。"""
        self.open()
        return self

    def __exit__(self, *_: object) -> None:
        """无论刷新是否抛出异常都关闭传输。"""
        self.close()

    def refresh_once(self) -> MotorFeedback:
        """请求并返回一条新接收的目标电机反馈。

        每次调用仅写入完整的状态刷新请求帧。读到噪声、无关帧或未完成帧时继续在
        本次总超时内读取；空读取视为底层读取已经等待到可用超时。

        Raises:
            TransportClosedError: 调用方尚未显式打开传输。
            ShortWriteError: 传输没有接受完整的 30 字节请求帧。
            StateRefreshTimeoutError: 在配置总时限内未获得目标反馈。
        """
        if not self._transport.is_open:
            raise TransportClosedError("状态刷新传输尚未打开")

        request = self._protocol.make_feedback_request(self._config.motor_id)
        written = self._transport.write(request, timeout_s=self._config.timeout_s)
        if written != len(request):
            raise ShortWriteError(
                f"状态刷新请求短写：期望 {len(request)} 字节，实际 {written} 字节"
            )

        return self._receiver.receive_feedback()
