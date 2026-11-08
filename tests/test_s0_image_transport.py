import sys
from types import SimpleNamespace as N

from examples.s0_image_transport import SCOPE, ExperimentalImageTransportMixin, image_subscription_qos


def fake_ros(monkeypatch):
    qos = N(DurabilityPolicy=N(VOLATILE='volatile'), HistoryPolicy=N(KEEP_LAST='keep_last'),
            ReliabilityPolicy=N(RELIABLE='reliable'), QoSProfile=lambda **fields:N(**fields))
    monkeypatch.setitem(sys.modules,'rclpy',N(qos=qos))
    monkeypatch.setitem(sys.modules,'rclpy.qos',qos)


def runtime(scope=SCOPE,simulated=True):
    return N(identity=(scope,'hash'),adapter=N(simulated_estimator=simulated,
        native=lambda canonical:'/simulator_only/oak/depth/image'),
        topics={'/simulator_only/oak/depth/image':'sensor_msgs/msg/Image'})


def test_only_experimental_native_depth_image_changes_to_reliable_latest(monkeypatch):
    fake_ros(monkeypatch); original=object(); rt=runtime()
    result=image_subscription_qos(rt,'/simulator_only/oak/depth/image',original)
    assert vars(result)==dict(depth=1,history='keep_last',reliability='reliable',durability='volatile')
    for topic in ('/fmu/out/vehicle_odometry','/clock','/tf_static','/simulator_only/oak/depth/camera_info'):
        assert image_subscription_qos(rt,topic,original) is original
    for other in (None,runtime(scope='ordinary'),runtime(simulated=False)):
        assert image_subscription_qos(other,'/simulator_only/oak/depth/image',original) is original


def test_subscription_preserves_original_callback_type_options_and_message(monkeypatch):
    fake_ros(monkeypatch)
    class Base:
        def create_subscription(self,*args,**kwargs): return args,kwargs
    class Owner(ExperimentalImageTransportMixin,Base): pass
    owner=Owner(); owner._hybrid_runtime=runtime(); seen=[]
    callback=seen.append; message=object(); msg_type=object()
    args,kwargs=owner.create_subscription(msg_type,'/simulator_only/oak/depth/image',callback,object(),raw=False)
    assert args[0] is msg_type and args[2] is callback and kwargs=={'raw':False}
    args[2](message)
    assert seen==[message]
