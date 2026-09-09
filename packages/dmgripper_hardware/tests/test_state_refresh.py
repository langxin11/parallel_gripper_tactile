"""验证无设备状态下的只读状态刷新流程。"""

from __future__ import annotations

import math

import pytest

from dmgripper_hardware import (
    DEFAULT_USB2CAN_BAUD_RATE,
    DmStateRefresher,
    FakeTransport,
    MotorFeedback,
    MotorLimits,
    ShortWriteError,
    StateRefreshTimeoutError,
    TransportClosedError,
    Usb2CanDeviceConfig,
    Usb2CanProtocol,
)


@pytest.fixture
def protocol() -> Usb2CanProtocol:
    """提供明确量程的离线协议实例。"""
    return Usb2CanProtocol(MotorLimits(-1.7, 1.7, -8.0, 8.0, -4.0, 4.0))


@pytest.fixture
def config() -> Usb2CanDeviceConfig:
    """提供不对应真实设备的严格配置。"""
    return Usb2CanDeviceConfig("fake://usb2can", motor_id=1, master_id=17, timeout_s=0.1)


def test_device_config_uses_documented_default_baud_rate() -> None:
    """未覆盖波特率时固定使用 USB2CAN 的保守参考默认值。"""
    config = Usb2CanDeviceConfig("fake://usb2can", motor_id=1, master_id=17, timeout_s=0.1)

    assert config.baud_rate == DEFAULT_USB2CAN_BAUD_RATE == 921600


@pytest.mark.parametrize(
    ("kwargs", "message"),
    [
        ({"port": " "}, "端点"),
        ({"port": "fake\x00port"}, "空字节"),
        ({"baud_rate": 0}, "波特率"),
        ({"motor_id": 0}, "电机"),
        ({"master_id": 0}, "主机"),
        ({"master_id": 1}, "必须不同"),
        ({"timeout_s": 0.0}, "超时"),
        ({"timeout_s": math.inf}, "超时"),
    ],
)
def test_device_config_rejects_unsafe_or_ambiguous_values(
    kwargs: dict[str, object], message: str
) -> None:
    """端点、ID 与超时的非法值应在任何 I/O 前失败。"""
    values: dict[str, object] = {
        "port": "fake://usb2can",
        "motor_id": 1,
        "master_id": 17,
        "timeout_s": 0.1,
    }
    values.update(kwargs)

    with pytest.raises(ValueError, match=message):
        Usb2CanDeviceConfig(**values)  # type: ignore[arg-type]


def test_refresh_handles_partial_noise_and_unrelated_frames(
    config: Usb2CanDeviceConfig, protocol: Usb2CanProtocol
) -> None:
    """刷新会在同一时限内重组分段字节、跳过噪声与其他 ID 的反馈。"""
    transport = FakeTransport()
    refresher = DmStateRefresher(config, protocol, transport)
    target = protocol.encode_feedback_frame(1, MotorFeedback(0.5, 1.0, 0.5, 1))
    unrelated = protocol.encode_feedback_frame(42, MotorFeedback(0.0, 0.0, 0.0, 0))

    assert transport.is_open is False
    assert transport.written_payloads == []
    refresher.open()
    transport.inject_received(b"noise" + unrelated[:10])
    transport.inject_received(unrelated[10:] + target[:4])
    transport.inject_received(target[4:])

    feedback = refresher.refresh_once()

    assert feedback.position_rad == pytest.approx(0.5, abs=3.4 / 65535)
    assert feedback.velocity_rad_s == pytest.approx(1.0, abs=16.0 / 4095)
    assert len(transport.written_payloads) == 1
    assert transport.written_payloads[0] == protocol.make_feedback_request(1)


def test_refresh_rejects_closed_transport_without_writing(
    config: Usb2CanDeviceConfig, protocol: Usb2CanProtocol
) -> None:
    """状态刷新绝不为便利而隐式打开传输。"""
    transport = FakeTransport()
    refresher = DmStateRefresher(config, protocol, transport)

    with pytest.raises(TransportClosedError, match="尚未打开"):
        refresher.refresh_once()

    assert transport.written_payloads == []


def test_refresh_raises_timeout_for_empty_read(
    config: Usb2CanDeviceConfig, protocol: Usb2CanProtocol
) -> None:
    """底层超时空读应转换为明确的状态反馈超时。"""
    transport = FakeTransport()
    refresher = DmStateRefresher(config, protocol, transport)
    refresher.open()

    with pytest.raises(StateRefreshTimeoutError, match="超时"):
        refresher.refresh_once()

    assert transport.written_payloads == [protocol.make_feedback_request(1)]


class ShortWriteTransport(FakeTransport):
    """以可控短写模拟 USB／串口拥塞。"""

    def write(self, payload: bytes) -> int:
        """记录后故意少报告一个已接受字节。"""
        super().write(payload)
        return len(payload) - 1


def test_refresh_rejects_short_request_write(
    config: Usb2CanDeviceConfig, protocol: Usb2CanProtocol
) -> None:
    """只有完整请求帧才允许进入读取阶段。"""
    transport = ShortWriteTransport()
    refresher = DmStateRefresher(config, protocol, transport)
    refresher.open()

    with pytest.raises(ShortWriteError, match="短写"):
        refresher.refresh_once()

    assert transport.written_payloads == [protocol.make_feedback_request(1)]


def test_context_manager_closes_transport_after_refresh_exception(
    config: Usb2CanDeviceConfig, protocol: Usb2CanProtocol
) -> None:
    """异常路径也必须关闭由上下文管理器打开的传输。"""
    transport = FakeTransport()
    refresher = DmStateRefresher(config, protocol, transport)

    with pytest.raises(StateRefreshTimeoutError):
        with refresher:
            refresher.refresh_once()

    assert transport.is_open is False
