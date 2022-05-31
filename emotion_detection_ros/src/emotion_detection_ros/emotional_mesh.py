import numpy as np
import math
import cv2

class EmotionalMesh:
    def __init__(self, frame_shape):
        # Source frame
        self.frame_shape = frame_shape

        # Indexes of 'Emotional Mesh' in  Mediapipe
        self.indexes = [61, 291, 0, 17, 50, 280, 48, 4, 278,
            206, 426, 133, 130, 159, 145, 362, 359, 386, 374, 122,
            351, 46, 105, 107, 276, 334, 336]
        
        # Coordinates of 'Emotional Mesh' - Tuple (x, y)
        self.coordinates = []

        # Relative coordinates in source frame of 'Emotional Mesh' - Tuple (x, y)
        self.rcoordinates = []

        # Bounding box in source frame
        # Format: [cx_min, cy_min, cx_max, cy_max]
        self.bounding_box = np.zeros(4)

        # Angles
        # Format: [angle1, angle2, angle3, ...]
        self.num_angles = 21
        self.angles = np.zeros((1, self.num_angles))

    def update_landmarks(self, face_landmarks):
        height, width, _ = self.frame_shape
        for index in self.indexes:
            x = face_landmarks.landmark[index].x
            y = face_landmarks.landmark[index].y
            self.coordinates.append((x, y))
            self.rcoordinates.append((int(x*width), int(y*height)))
        self.__calculate_angles()
        self.__calculate_bounding_box(face_landmarks)

    def draw(self, frame):
        for coord in self.rcoordinates:
            cv2.circle(frame, (coord[0], coord[1]), 2, (0, 255, 0), -1)

    # Private methods ----------------------------------------------------------------
    def __distance(self, point1, point2):
        x0 = point1[0]
        y0 = point1[1]
        x1 = point2[0]
        y1 = point2[1]
        return math.sqrt((x0 - x1)**2+(y0 - y1)**2)

    def __angle(self, point1, point2, point3):
        side1 = self.__distance(point2, point3)
        side2 = self.__distance(point1, point3)
        side3 = self.__distance(point1, point2)
        
        angle = math.degrees(math.acos((side1**2+side3**2-side2**2)/(2*side1*side3)))
        return angle

    def __calculate_bounding_box(self, face_landmarks):
        h, w, _ = self.frame_shape
        cx_min = w
        cy_min = h
        cx_max = cy_max = 0
        for id, lm in enumerate(face_landmarks.landmark):
            cx, cy = int(lm.x * w), int(lm.y * h)
            if cx < cx_min:
                cx_min = cx
            if cy < cy_min:
                cy_min = cy
            if cx > cx_max:
                cx_max = cx
            if cy > cy_max:
                cy_max = cy
        self.bounding_box = [cx_min, cy_min, cx_max, cy_max]
    
    def __calculate_angles(self):
        index = 0

        # Angle 0
        self.angles[0][index] = self.__angle(self.coordinates[7], self.coordinates[1], 
            self.coordinates[2])
        index += 1
        # Angle 1
        self.angles[0][index] = self.__angle(self.coordinates[2], self.coordinates[1], 
            self.coordinates[3])
        index += 1
        # Angle 2
        self.angles[0][index] = self.__angle(self.coordinates[0], self.coordinates[2], 
            self.coordinates[1])
        index += 1
        # Angle 3
        self.angles[0][index] = self.__angle(self.coordinates[1], self.coordinates[7], 
            self.coordinates[8])
        index += 1
        # Angle 4
        self.angles[0][index] = self.__angle(self.coordinates[0], self.coordinates[7], 
            self.coordinates[1])
        index += 1
        # Angle 5
        self.angles[0][index] = self.__angle(self.coordinates[8], self.coordinates[5], 
            self.coordinates[1])
        index += 1
        # Angle 6
        self.angles[0][index] = self.__angle(self.coordinates[8], self.coordinates[10], 
            self.coordinates[1])
        index += 1
        # Angle 7
        self.angles[0][index] = self.__angle(self.coordinates[18], self.coordinates[5], 
            self.coordinates[8])
        index += 1
        # Angle 8
        self.angles[0][index] = self.__angle(self.coordinates[8], self.coordinates[7], 
            self.coordinates[20])
        index += 1
        # Angle 9
        self.angles[0][index] = self.__angle(self.coordinates[26], self.coordinates[7], 
            self.coordinates[23])
        index += 1
        # Angle 10
        self.angles[0][index] = self.__angle(self.coordinates[7], self.coordinates[20], 
            self.coordinates[18])
        index += 1
        # Angle 11
        self.angles[0][index] = self.__angle(self.coordinates[20], self.coordinates[18], 
            self.coordinates[5])
        index += 1
        # Angle 12
        self.angles[0][index] = self.__angle(self.coordinates[17], self.coordinates[16], 
            self.coordinates[18])
        index += 1
        # Angle 13
        self.angles[0][index] = self.__angle(self.coordinates[26], self.coordinates[25], 
            self.coordinates[24])
        index += 1
        # Angle 14
        self.angles[0][index] = self.__angle(self.coordinates[20], self.coordinates[26], 
            self.coordinates[25])
        index += 1
        # Angle 15
        self.angles[0][index] = self.__angle(self.coordinates[25], self.coordinates[24], 
            self.coordinates[16])
        index += 1
        # Angle 16
        self.angles[0][index] = self.__angle(self.coordinates[24], self.coordinates[16], 
            self.coordinates[17])
        index += 1
        # Angle 17
        self.angles[0][index] = self.__angle(self.coordinates[18], self.coordinates[20], 
            self.coordinates[26])
        index += 1
        # Angle 18
        self.angles[0][index] = self.__angle(self.coordinates[5], self.coordinates[1], 
            self.coordinates[10])
        index += 1
        # Angle 19
        self.angles[0][index] = self.__angle(self.coordinates[10], self.coordinates[1], 
            self.coordinates[7])
        index += 1
        # Angle 20
        self.angles[0][index] = self.__angle(self.coordinates[10], self.coordinates[8], 
            self.coordinates[5])