"""只读 DM 电机状态刷新的运行时。"""

from __future__ import annotations

import time
from collections.abc import Callable

from .config import Usb2CanDeviceConfig
from .protocol import USB2CAN_RX_FRAME_SIZE, MotorFeedback, Usb2CanFrameExtractor, Usb2CanProtocol
from .transport import ByteTransport, TransportClosedError


class StateRefreshTimeoutError(TimeoutError):
    """在配置的状态刷新时限内未收到目标电机的有效反馈。"""


class ShortWriteError(OSError):
    """USB2CAN 未接受完整状态刷新请求帧。"""


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
    """

    def __init__(
        self,
        config: Usb2CanDeviceConfig,
        protocol: Usb2CanProtocol,
        transport: ByteTransport,
        *,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        """保存依赖，不打开端口或写入任何帧。"""
        self._config = config
        self._protocol = protocol
        self._transport = transport
        self._clock = clock
        self._extractor = Usb2CanFrameExtractor()

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
        written = self._transport.write(request)
        if written != len(request):
            raise ShortWriteError(
                f"状态刷新请求短写：期望 {len(request)} 字节，实际 {written} 字节"
            )

        deadline = self._clock() + self._config.timeout_s
        while True:
            remaining = deadline - self._clock()
            if remaining <= 0.0:
                raise StateRefreshTimeoutError("等待目标电机状态反馈超时")
            received = self._transport.read(USB2CAN_RX_FRAME_SIZE, timeout_s=remaining)
            if not received:
                raise StateRefreshTimeoutError("等待目标电机状态反馈超时")
            for frame in self._extractor.feed(received):
                feedback = self._protocol.parse_feedback(
                    frame,
                    motor_id=self._config.motor_id,
                    master_id=self._config.master_id,
                )
                if feedback is not None:
                    return feedback
