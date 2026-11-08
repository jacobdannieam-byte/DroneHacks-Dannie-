"""Reliable, latest-image transport only for the isolated S0 experiment."""

SCOPE = 's0_participant_map_unqualified_sitl'


def image_subscription_qos(runtime, topic, original):
    """Match the native reliable image writer without changing PX4/TF QoS.

    Reliable delivery retransmits lost image fragments; KEEP_LAST 1 bounds the
    reader queue. Original capture stamps and downstream freshness remain intact.
    This changes delivery, not the camera rate or the spatial processing budget.
    """
    if (runtime is None or not getattr(runtime.adapter, 'simulated_estimator', False)
            or not getattr(runtime, 'identity', ()) or runtime.identity[0] != SCOPE
            or topic != runtime.adapter.native('/camera/depth/image_raw')
            or runtime.topics.get(topic) != 'sensor_msgs/msg/Image'):
        return original
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    return QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                      reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)


class ExperimentalImageTransportMixin:
    def create_subscription(self, msg_type, topic, callback, qos_profile, **kwargs):
        qos = image_subscription_qos(getattr(self, '_hybrid_runtime', None), topic, qos_profile)
        return super().create_subscription(msg_type, topic, callback, qos, **kwargs)
