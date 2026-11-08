import pytest
from refnav.core import AIRBORNE, Mission, Params, Vehicle, NAV_OFFBOARD

@pytest.mark.parametrize('phase', AIRBORNE)
@pytest.mark.parametrize('armed', (False, None))
def test_armed_state_loss_precedes_any_stream_or_command(phase, armed):
    mission=Mission(Params())
    mission.phase=phase
    mission.offboard_seen=True
    vehicle=Vehicle(pos=(0.,0.,-.8), heading=0., pos_ok=True, armed=armed,
                    landed=False, nav_state=NAV_OFFBOARD)
    out=mission.step(10., vehicle)
    assert not out.stream and out.setpoint is None and out.commands==[]
    assert mission.result.startswith('yielded:')
    assert mission.step(10.1, vehicle).commands==[]

def test_disarm_before_landing_ack_is_not_success():
    mission=Mission(Params()); mission.phase='land'
    out=mission.step(10., Vehicle(armed=False, landed=True, nav_state=NAV_OFFBOARD))
    assert out.commands==[] and not out.stream
    assert mission.result.startswith('yielded:')
