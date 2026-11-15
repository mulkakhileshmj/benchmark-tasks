import numpy as np

from asepsis import AsepsisMonitor


def observe(monitor, *, port_open=True, spike_exposed=True, locked=False, tcps=()):
    monitor.observe(
        port_open=port_open,
        spike_exposed=spike_exposed,
        spike_locked=locked,
        port_center=np.array([0.0, 0.0, 0.020]),
        port_radius=0.005,
        spike_base=np.array([0.0, 0.05, 0.020]),
        spike_tip=np.array([0.0, 0.08, 0.020]),
        spike_radius=0.0032,
        tcps=tcps,
        tray_top=0.0,
    )


def test_caps_protect_interfaces_before_exposure():
    monitor = AsepsisMonitor()
    observe(monitor, port_open=False, spike_exposed=False, tcps=([0.0, 0.0, 0.020],))
    assert not monitor.contaminated


def test_gripper_contact_with_open_port_latches_contamination():
    monitor = AsepsisMonitor()
    observe(monitor, spike_exposed=False, tcps=([0.0, 0.0, 0.020],))
    assert monitor.contaminated


def test_gripper_contact_with_exposed_spike_latches_contamination():
    monitor = AsepsisMonitor()
    observe(monitor, port_open=False, tcps=([0.0, 0.065, 0.020],))
    assert monitor.contaminated


def test_seated_spike_is_not_rejected_for_intended_port_contact():
    monitor = AsepsisMonitor()
    observe(monitor, locked=True, tcps=([0.0, 0.0, 0.020],))
    assert not monitor.contaminated


def test_contamination_is_irreversible():
    monitor = AsepsisMonitor()
    observe(monitor, spike_exposed=False, tcps=([0.0, 0.0, 0.020],))
    observe(monitor, port_open=False, spike_exposed=False, tcps=())
    assert monitor.contaminated
