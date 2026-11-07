"""Keeps the camera's automatic exposure capped, so camera tracking keeps sharp pictures.

How it fits: mission_node.py creates one CameraExposureCap at startup and calls check() every few
seconds. It changes the camera driver's settings over ROS, through the two standard services every
ROS program offers for its parameters (<node>/set_parameters and <node>/get_parameters). It makes no
flight decisions.

Why not rclpy's AsyncParameterClient, which wraps those same two services: it only exists from ROS 2
Iron onward. Drone 16 runs ROS 2 Humble, where importing it fails (found by running the mission in
the simulator, which has the drone's Humble). CameraParameterServices below does the same job.

Why: with automatic exposure the camera picks a longer exposure in dim light. A long exposure
blurs the picture while the drone moves, and camera tracking (the drone's only source of
horizontal position) can then lose track. The organizer code audit says to cap the automatic
exposure at 8000 microseconds on both cameras of the pair (left and right).

How, following the audit: one setting at a time, each sent only after the camera has answered
the previous one; the numbers first, then the switches:
  left and right r_auto_exposure_limit = 8000
  left and right r_set_auto_exposure_limit = true    (use that limit)
  left and right r_set_man_exposure = false          (keep exposure automatic)
If the camera program restarts, its settings go back to their defaults. So check() applies the cap
again when the camera's settings service comes back, and also whenever reading the settings back
shows they differ from the cap.

What it does not do: wait for anything. Every request is answered later, through a callback, so
the mission keeps running at full rate.
"""
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.parameter import Parameter

# The type numbers ROS uses inside a parameter value message (rcl_interfaces/ParameterType).
ROS_PARAMETER_TYPE_BOOL = 1
ROS_PARAMETER_TYPE_INTEGER = 2
ROS_PARAMETER_TYPE_DOUBLE = 3
ROS_PARAMETER_TYPE_STRING = 4


def camera_settings_in_order(auto_exposure_limit_microseconds):
    """(name, value) pairs, in the order they must be applied: numbers first, then switches."""
    return [
        ("left.r_auto_exposure_limit", int(auto_exposure_limit_microseconds)),
        ("right.r_auto_exposure_limit", int(auto_exposure_limit_microseconds)),
        ("left.r_set_auto_exposure_limit", True),
        ("right.r_set_auto_exposure_limit", True),
        ("left.r_set_man_exposure", False),
        ("right.r_set_man_exposure", False),
    ]


def python_value(parameter_value):
    """The plain Python value inside a ROS parameter value message, or None for other types."""
    kind = parameter_value.type
    if kind == ROS_PARAMETER_TYPE_BOOL:
        return parameter_value.bool_value
    if kind == ROS_PARAMETER_TYPE_INTEGER:
        return parameter_value.integer_value
    if kind == ROS_PARAMETER_TYPE_DOUBLE:
        return parameter_value.double_value
    if kind == ROS_PARAMETER_TYPE_STRING:
        return parameter_value.string_value
    return None


class CameraParameterServices:
    """The camera driver's two parameter services, with the same three calls as AsyncParameterClient."""

    def __init__(self, node, camera_node_name):
        self.set_service = node.create_client(SetParameters, f"{camera_node_name}/set_parameters")
        self.get_service = node.create_client(GetParameters, f"{camera_node_name}/get_parameters")

    def services_are_ready(self):
        return self.set_service.service_is_ready() and self.get_service.service_is_ready()

    def set_parameters(self, parameters):
        request = SetParameters.Request()
        request.parameters = [parameter.to_parameter_msg() for parameter in parameters]
        return self.set_service.call_async(request)

    def get_parameters(self, names):
        request = GetParameters.Request()
        request.names = list(names)
        return self.get_service.call_async(request)


class CameraExposureCap:
    def __init__(self, node, camera_node_name, auto_exposure_limit_microseconds):
        self.logger = node.get_logger()
        self.camera_node_name = camera_node_name
        self.client = CameraParameterServices(node, camera_node_name)
        self.wanted = camera_settings_in_order(auto_exposure_limit_microseconds)
        self.state = "waiting_for_camera"     # then "applying", then "applied"; "retry" after a refusal
        self.still_to_send = []
        self.request_in_flight = False

    def check(self):
        """Apply the cap when the camera appears, and again if it lost it."""
        if self.request_in_flight:
            return
        if not self.client.services_are_ready():
            if self.state != "waiting_for_camera":
                self.logger.warn(f"The camera's settings service on {self.camera_node_name} has gone (the camera "
                                 "program stopped?). The exposure cap will be applied again when it comes back.")
            self.state = "waiting_for_camera"
            return
        if self.state in ("waiting_for_camera", "retry"):
            self.start_applying("the camera's settings service is available")
        elif self.state == "applied":
            self.read_back()

    def start_applying(self, why):
        self.logger.info(f"Applying the camera exposure cap on {self.camera_node_name}, because {why}.")
        self.state = "applying"
        self.still_to_send = list(self.wanted)
        self.send_next()

    def send_next(self):
        if not self.still_to_send:
            self.state = "applied"
            self.logger.info(f"Camera exposure cap applied on {self.camera_node_name}: "
                             + ", ".join(f"{name} = {value}" for name, value in self.wanted))
            return
        name, value = self.still_to_send[0]
        self.request_in_flight = True
        future = self.client.set_parameters([Parameter(name, value=value)])
        future.add_done_callback(self.on_setting_answered)

    def on_setting_answered(self, future):
        self.request_in_flight = False
        name, value = self.still_to_send.pop(0)
        refusal = self.refusal(future)
        if refusal is not None:
            self.logger.warn(f"The camera did not accept {name} = {value} ({refusal}). The whole exposure cap will "
                             "be applied again at the next check.")
            self.state = "retry"
            self.still_to_send = []
            return
        self.send_next()

    @staticmethod
    def refusal(future):
        """Why a set request failed, or None if the camera accepted it."""
        try:
            response = future.result()
        except Exception as error:            # the request itself failed, for example a timeout
            return f"the request failed: {error}"
        if response is None or not response.results:
            return "no answer"
        if not response.results[0].successful:
            return response.results[0].reason or "refused without a reason"
        return None

    def read_back(self):
        self.request_in_flight = True
        future = self.client.get_parameters([name for name, _ in self.wanted])
        future.add_done_callback(self.on_read_back)

    def on_read_back(self, future):
        self.request_in_flight = False
        try:
            values = [python_value(value) for value in future.result().values]
        except Exception:                      # no answer: try again at the next check
            return
        if values != [value for _, value in self.wanted]:
            self.start_applying("its settings no longer match the cap (the camera program restarted?)")
