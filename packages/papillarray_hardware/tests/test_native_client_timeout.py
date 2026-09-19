"""原厂租期临近时客户端读取预算及状态查询不被串口写锁住。"""

from threading import Event, Thread

import numpy as np
import pytest

from papillarray_hardware import PapillArraySerialClient, PapillArraySerialConfig, PtsPacket
from papillarray_hardware.native_slip import NativeSlipLease


class _Port:
    """已打开且可调读取超时的最小假串口；实际读取由注入的 Reader 接管。"""

    is_open = True

    def __init__(self) -> None:
        """初始化可被客户端临时改写的底层读取超时。"""
        self.timeout: float | None = None

    def close(self) -> None:
        """响应客户端退出时的关闭。"""
        self.is_open = False


def _packet() -> PtsPacket:
    """构造满足双侧数量互锁校验的最小观测包。"""
    force = np.empty((0, 3), dtype=np.float64)
    return PtsPacket(
        packet_counter=1,
        timestamp_us=2,
        pillar_forces=[force.copy(), force.copy()],
        pillar_displacements=[force.copy(), force.copy()],
        global_forces=[np.zeros(3, dtype=np.float64) for _ in range(2)],
        global_torques=[np.zeros(3, dtype=np.float64) for _ in range(2)],
    )


@pytest.mark.parametrize("fails", [False, True])
def test_client_shortens_serial_timeout_and_restores_it(fails):
    """临近截止缩短包等待和底层读取预算，异常返回也恢复原配置。"""
    port = _Port()
    port.timeout = 1.0
    client = PapillArraySerialClient(PapillArraySerialConfig(), lambda *_: port)

    class Reader:
        """检查读取发生时的真实串口预算。"""

        def read_packet(self, packet_timeout_s):
            assert packet_timeout_s == pytest.approx(0.012)
            assert port.timeout == pytest.approx(0.012)
            if fails:
                raise TimeoutError("测试截止")
            return _packet()

    with client:
        client._reader = Reader()
        if fails:
            with pytest.raises(TimeoutError):
                client.read_packet(packet_timeout_s=0.012)
        else:
            assert client.read_packet(packet_timeout_s=0.012).n_sensors == 2
        assert port.timeout == 1.0


def test_serial_write_does_not_hold_state_lock():
    """启停写入等待时，控制线程仍能查询状态和提交停止。"""
    lease = NativeSlipLease()
    entered, release, queried = Event(), Event(), Event()

    class Client:
        """显式阻塞命令写入，不访问设备。"""

        def start_slip_detection(self):
            entered.set()
            release.wait(1.0)

        def stop_slip_detection(self):
            pass

    lease.request_start(1, max_duration_s=1.0, confirmation_timeout_s=0.2)
    sender = Thread(target=lambda: lease.tick(Client(), 0.0), daemon=True)

    def query():
        assert lease.status.phase == "starting"
        lease.request_stop("interrupted")
        queried.set()

    sender.start()
    try:
        assert entered.wait(0.5)
        reader = Thread(target=query, daemon=True)
        reader.start()
        assert queried.wait(0.5)
        reader.join(0.5)
    finally:
        release.set()
        sender.join(0.5)
