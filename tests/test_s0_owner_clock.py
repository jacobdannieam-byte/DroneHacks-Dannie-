from copy import deepcopy
from types import SimpleNamespace as N

from examples.s0_owner_clock import ExperimentalOwnerClockMixin, WAIT_NS
from onboard.diagnostics.phase1_probe import RESET_FIELDS


class Base:
    def __init__(self):
        self.now = self.wall = 1_000_000_000
        self.v = N(pos=None, pos_ok=False, armed=None, nav_state=None, landed=None, q=None,
                   attitude_capture_ns=None, attitude_received_ns=None, attitude_reset_counter=None,
                   landed_capture_ns=None)
        self._hybrid_reports = {}
        self._yaw_align = (False,None)
        self.invalidations = 0
        self.delivered = []
        mapping = N(offset_ns=0,max_error_ns=0,valid_from_ns=0,valid_until_ns=10_000_000_000)
        self._hybrid_runtime = N(loop=N(replay=N(hybrid=N(clock=mapping))))
        self.mission = N(yield_hybrid_authority=lambda now,reason: ('yield',reason))

    def get_clock(self): return N(now=lambda: N(nanoseconds=self.now))
    def _owner_clock_wall(self): return self.wall
    def _invalidate_hybrid(self): self.invalidations += 1
    def _on_lpos(self,m):
        self.v.pos=(m.x,m.y,m.z); self.v.pos_ok=True
        self._hybrid_reports['position']=(m.timestamp_sample*1000,tuple(getattr(m,k) for k in RESET_FIELDS))
        self.delivered.append(m)
    def _on_status(self,m):
        self.v.armed=m.arming_state==2; self.v.nav_state=m.nav_state
        self._hybrid_reports['authority']=(m.timestamp*1000,(self.v.armed,self.v.nav_state))
        self.delivered.append(m)
    def _on_land(self,m): self.v.landed=m.landed; self.delivered.append(m)
    def _on_att(self,m): self.v.q=m.q; self.v.attitude_reset_counter=m.quat_reset_counter; self.delivered.append(m)
    def _on_est_flags(self,m): self._yaw_align=(m.cs_yaw_align,self.now); self.delivered.append(m)
    def _runtime_owner(self,now): return 'owner'
    def _hybrid_watchdog_tick(self): return 'watchdog'


class Owner(ExperimentalOwnerClockMixin,Base):
    def _owner_clock_wall(self): return self.wall


def position(stamp=1_004_000,reset=0):
    return N(timestamp=stamp,timestamp_sample=stamp,x=1.,y=2.,z=3.,heading=0.,xy_valid=True,z_valid=True,heading_good_for_control=True,
             **{key:reset for key in RESET_FIELDS})


def test_original_full_message_waits_until_clock_catches_up():
    owner=Owner(); message=position(); original=deepcopy(vars(message))
    owner._on_lpos(message)
    assert owner.v.pos is None and not owner.delivered
    owner.now += 4_000_000; owner.wall += 4_000_000
    assert owner._runtime_owner(owner.now)=='owner'
    assert owner.delivered==[message] and owner.delivered[0] is message
    assert vars(message)==original and owner._hybrid_reports['position'][0]==1_004_000_000
    audit=owner._owner_clock_audit[0]
    assert audit['original_receipt_ns']==1_000_000_000 and audit['released_ros_ns']==1_004_000_000


def test_future_authority_change_revokes_old_state_before_delivery():
    owner=Owner(); owner._on_status(N(timestamp=1_000_000,arming_state=2,nav_state=14))
    owner._on_status(N(timestamp=1_004_000,arming_state=1,nav_state=3))
    assert owner.v.armed is None and owner.v.nav_state is None and 'authority' not in owner._hybrid_reports
    owner.now+=4_000_000; owner.wall+=4_000_000; owner._owner_clock_drain()
    assert owner.v.armed is False and owner.v.nav_state==3


def test_new_reset_discards_pending_old_reset_and_invalidates_position():
    owner=Owner(); owner._on_lpos(position(1_000_000)); owner._on_lpos(position(1_004_000))
    owner._on_lpos(position(1_008_000,reset=1))
    assert not owner.v.pos_ok and 'position' not in owner._hybrid_reports
    assert len(owner._owner_clock_pending)==1
    owner.now+=8_000_000; owner.wall+=8_000_000; owner._owner_clock_drain()
    assert len(owner.delivered)==2 and owner._hybrid_reports['position'][1]==(1,)*len(RESET_FIELDS)


def test_pause_expires_on_steady_watchdog_and_fault_cannot_reopen():
    owner=Owner(); owner._on_lpos(position()); owner.wall+=WAIT_NS+1
    owner._hybrid_watchdog_tick()
    assert owner._owner_clock_fault=='experimental_owner_clock_wait_expired'
    owner.now+=10_000_000; owner._on_lpos(position(1_010_000))
    assert not owner.delivered and owner._runtime_owner(owner.now)[0]=='yield'


def test_rewind_and_beyond_existing_fifty_ms_bound_refuse():
    owner=Owner(); owner._on_lpos(position(1_050_001))
    assert owner._owner_clock_fault and not owner.delivered
    owner=Owner(); owner._on_lpos(position()); owner.now-=1
    owner._owner_clock_drain()
    assert owner._owner_clock_fault=='experimental_owner_clock_rewind'


def test_future_invalid_position_does_not_retain_previous_admission():
    owner=Owner(); owner._on_lpos(position(1_000_000))
    bad=position(); bad.xy_valid=False
    owner._on_lpos(bad)
    assert owner._owner_clock_fault and not owner.v.pos_ok and not owner._hybrid_reports


def test_attitude_and_landed_payloads_are_not_mutated_before_catchup():
    owner=Owner()
    owner._on_att(N(timestamp=1_004_000,timestamp_sample=1_004_000,q=[1.,0.,0.,0.],quat_reset_counter=0))
    owner._on_land(N(timestamp=1_004_000,landed=True))
    assert owner.v.q is None and owner.v.landed is None
    owner.now+=4_000_000; owner.wall+=4_000_000; owner._owner_clock_drain()
    assert owner.v.q==[1.,0.,0.,0.] and owner.v.landed is True


def test_same_source_conflicting_position_refuses_but_identical_duplicate_is_ok():
    owner=Owner(); message=position(1_000_000)
    owner._on_lpos(message); owner._on_lpos(deepcopy(message))
    assert not owner._owner_clock_fault
    conflict=deepcopy(message); conflict.x+=.01
    owner._on_lpos(conflict)
    assert 'conflicting' in owner._owner_clock_fault and not owner.v.pos_ok


def test_clock_uncertainty_waits_for_upper_bound_and_startup_suppresses_output():
    owner=Owner(); owner._hybrid_runtime.loop.replay.hybrid.clock.max_error_ns=2_000_000
    owner.mission.phase='arm'
    owner._on_status(N(timestamp=1_004_000,arming_state=2,nav_state=14))
    owner.now+=4_000_000; owner.wall+=4_000_000
    out=owner._runtime_owner(owner.now)
    assert not out.stream and not out.commands and not owner.delivered
    owner.now+=2_000_000; owner.wall+=2_000_000
    assert owner._runtime_owner(owner.now)=='owner' and owner.v.armed is True


def test_airborne_future_mode_loss_never_uses_startup_wait_exception():
    owner=Owner(); owner.mission.phase='go'
    owner._on_status(N(timestamp=1_000_000,arming_state=2,nav_state=14))
    owner._on_status(N(timestamp=1_004_000,arming_state=2,nav_state=3))
    assert owner._runtime_owner(owner.now)=='owner'  # Real RefNav now observes revoked authority.
    assert owner.v.armed is None and 'authority' not in owner._hybrid_reports
