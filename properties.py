
class Property():
    """
    Required heading values for car metrics and stl property monitor
    """
    headings={
        "time": "float",            # Timestamp in seconds, samplint time of signal
        "device_id": "integer",     # Unique vehicle id for car
        "speed": "float",           # Vehicle speed (m/s)
        "acceleration": "float",    # Vehicle accleration (m/s^2)
        "x": "float",               # X-coordinate position on treadmill
        "y": "float",               # Y-coordinate position on treadmill
        "angle": "float",           # Steering angle in degrees
    }
