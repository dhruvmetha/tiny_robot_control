"""Topic name constants for pub/sub messaging."""


class Topics:
    """Central registry of topic names."""

    SENSOR_SIM = "sensor.sim"  # SimSensorNode publishes
    SENSOR_VISION = "sensor.vision"  # ArucoObserver publishes Observation
    CAMERA_BGR_RAW = "camera.bgr.raw"  # Raw BGR frames from camera
    CAMERA_RGB_RAW = "camera.rgb.raw"  # Raw RGB frames (converted from BGR)
    CAMERA_BGR_PROCESSED = "camera.bgr.processed"  # Undistorted BGR for ArUco/OpenCV
    WORLD_STATE = "world.state"  # WorldState republishes
    COMMAND_ACTION = "command.action"  # Control loop publishes
    ROBOT_STATUS = "robot.status"  # RealEnv publishes SenderStatus
