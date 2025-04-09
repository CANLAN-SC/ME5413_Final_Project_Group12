import math
# ====================== Global Configuration Parameters ======================
class Config:
    # Topic name configuration
    TOPICS = {
        # Costmap related topics
        'GLOBAL_COSTMAP': '/move_base/global_costmap/costmap',
        'GLOBAL_COSTMAP_UPDATES': '/move_base/global_costmap/costmap_updates',
        'EXPLORE_COSTMAP': '/frontier_explore/costmap',
        'EXPLORE_COSTMAP_UPDATES': '/frontier_explore/costmap_updates',
        'FRONTIERS': '/explore/frontiers',
        
        # Detection trigger and result topics
        'BOX_DETECTION_TRIGGER': '/box_detection_trigger',
        'DETECTED_BOXES': '/detected_boxes',
        'DETECTED_BOUNDING_BOXES': '/detected_bounding_boxes',
        'BRIDGE_DETECTION_TRIGGER': '/bridge_detection_trigger',
        'DETECTED_BRIDGES': '/detected_bridges',
        'OCR_TRIGGER': '/ocr_trigger',
        'CMD_STOP': '/cmd_stop',
        'OPEN_BRIDGE': '/cmd_open_bridge',
        
        # Navigation related topics
        'MOVE_BASE': 'move_base',

        # Box extraction related topics
        'BOX_EXTRACTION': '/move_base/global_costmap/costmap',
        # OCR related topics
        'RECOGNIZED_DIGIT': '/recognized_digit',        
    }
    
    # Timeout settings (seconds)
    TIMEOUTS = {
        'INIT': 60.0,                # Initialization timeout
        'EXPLORE': 20.0,            # Exploration task timeout
        'BOX_DETECTION': 3.0,       # Box detection timeout
        'BRIDGE_DETECTION': 15.0,    # Bridge detection timeout
        'NAVIGATION': 30.0,          # Navigation timeout
        'OCR_PROCESSING': 1.0,       # OCR processing timeout
        'BRIDGE_OPEN': 5.0,          # Bridge opening wait time
    }

    # Obstacle threshold
    OBSTACLE_THRESHOLD = 99  # Initial dynamically adjustable threshold

    # Exploration area settings
    EXPLORE_MAP_BOUNDS = {
        'X_MIN': 9.0,    # Map X coordinate minimum value
        'X_MAX': 19.0,   # Map X coordinate maximum value
        'Y_MIN': -22.0,    # Map Y coordinate minimum value
        'Y_MAX': -2.0,   # Map Y coordinate maximum value
    }

    # River area settings
    RIVER_BOUNDS = {
        'X_MIN': 5.0,    # Map X coordinate minimum value
        'X_MAX': 10.0,   # Map X coordinate maximum value
        'Y_MIN': -22.0,    # Map Y coordinate minimum value
        'Y_MAX': -2.0,   # Map Y coordinate maximum value
    }

    # Goal area settings
    GOAL_MAP_BOUNDS = {
        'X_MIN': 0.0,    # Map X coordinate minimum value
        'X_MAX': 5.0,   # Map X coordinate maximum value
        'Y_MIN': -22.0,    # Map Y coordinate minimum value
        'Y_MAX': -2.0,   # Map Y coordinate maximum value
    }

    # Exploration related settings
    EXPLORE = {
        'FRONTIER_THRESHOLD': 0,     # Frontier point count threshold, below this value exploration is considered complete
        'CHECK_RATE': 0.5,           # Check frequency (Hz)
    }
    
    # Bridge related settings
    BRIDGE_LENGTH = 5.0  # Bridge length (meters)
    BRIDGE_X = 9.0  # Bridge X coordinate
    CROSSING_SPEED = 1.0  # Crossing speed (m/s)
    # Target position coordinate list (x, y, orientation.w)
    GOALS = [
        (0.0, -18.0, 1.0), # (0:5,-22:-2)
        (0.0, -14.0, 1.0),
        (0.0, -10.0, 1.0),
        (0.0, -6.0, 1.0)
    ]

    # Best viewing distance
    VIEWING_DISTANCE = 2.0

    # Viewing angles
    VIEWING_ANGLES = [0, math.pi/2, math.pi, 3*math.pi/2]  # 0°, 90°, 180°, 270°

    # Box side length
    BOX_SIZE = 0.5  # meters

    # Clustering parameters
    CLUSTERING = {
        'width': 35,
        'height': 35,
        'MIN_POINTS': 15,  # Minimum point count
        'MAX_POINTS': 200,  # Maximum point count
        'MIN_SAMPLES': 5,  # Minimum sample size
        'EPS': 5,  # DBSCAN eps parameter, smaller eps creates more, smaller clusters

        'width_bridge': 70,
        'height_bridge': 100,
        'MIN_POINTS_BRIDGE': 20,  # Minimum point count
        'MAX_POINTS_BRIDGE': 300,  # Maximum point count
        'MIN_SAMPLES_BRIDGE': 10,  # Minimum sample size
        'EPS_BRIDGE': 6,  # DBSCAN eps parameter, larger eps creates fewer, larger clusters
    }
