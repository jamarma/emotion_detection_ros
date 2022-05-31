import numpy as np

class Emotion:
    def __init__(self):
        # Predicted emotion and percentage
        self.label = None
        self.class_name = None
        self.probability = None
        
        # Bounding box in source frame
        # Format: [cx_min, cy_min, cx_max, cy_max]
        self.bounding_box = np.zeros(4)
