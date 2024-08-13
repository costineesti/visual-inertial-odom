import enum as Enum

from Brain.src.utils.messages.allMessages import *

# ================ Lane Detection =================
class LaneDetected(Enum):
    Queue = "General"
    Owner = "threadLaneDetection"
    msgID = 1
    msgType = "bool"

class NoLaneDetected(Enum):
    Queue = "General"
    Owner = "threadLaneDetection"
    msgID = 2
    msgType = "bool"

class LeftLine(Enum):
    Queue = "General"
    Owner = "threadLaneDetection"
    msgID = 3
    msgType = "numpy.ndarray"

class RightLine(Enum):
    Queue = "General"
    Owner = "threadLaneDetection"
    msgID = 4
    msgType = "numpy.ndarray"

class MiddleLine(Enum):
    Queue = "General"
    Owner = "threadLaneDetection"
    msgID = 5
    msgType = "numpy.ndarray"

class DisplayWindow(Enum):
    Queue = "General"
    Owner = "threadLaneDetection"
    msgID = 6
    msgType = "dict"
    
    # {"windows": {"left": value, "right": value}}
    
class DistanceErrorBottom(Enum):
    Queue = "General"
    Owner = "threadLaneDetection"
    msgID = 7
    msgType = "numpy.float64"

class DistanceErrorTop(Enum):
    Queue = "General"
    Owner = "threadLaneDetection"
    msgID = 8
    msgType = "numpy.float64"

class CameraPoints(Enum):
    Queue = "General"
    Owner = "threadLaneDetection"
    msgID = 8
    msgType = "dict"

# ================ Sign Detection ================
class SignDetected(Enum):
    Queue = "General"
    Owner = "threadSignDetection"
    msgID = 1
    msgType = "bool"

class SignType(Enum):
    Queue = "General"
    Owner = "threadSignDetection"
    msgID = 2
    msgType = "str"

class SignCoordinations(Enum):
    Queue = "General"
    Owner = "threadSignDetection"
    msgID = 3
    msgType = "list"

class SignImage(Enum):
    Queue = "General"
    Owner = "threadSignDetection"
    msgID = 4
    msgType = "numpy.ndarray"

# ================ Intersection Detection ================
class IntersectionDetected(Enum):
    Queue = "General"
    Owner = "threadIntersectionDetection"
    msgID = 1
    msgType = "dict"

    # {"distance": value, "sign": value, "line": value, "id": value}

# ================ Roi =================
class Rois(Enum):
    Queue = "General"
    Owner = "threadCameraCalibration"
    msgID = 1
    msgType = "dict"

    # {"Road": value, "Intersection": value, "Sign": value}

# ============== FROM NUCLEO ==================
class ImuData(Enum):
    Queue = "General"
    Owner = "ExtendedThreadRead"
    msgID = 2
    msgType = "String"

# ================ Visual Odometry ================
class VisualOdometry(Enum):
    Queue = "General"
    Owner = "threadVisualOdometry"
    msgID = 1
    msgType = "dict"

    # {"x": value, "y": value, "orientation": value}
