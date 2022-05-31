import mediapipe as mp
import cv2

from emotion_detection_ros.emotional_mesh import EmotionalMesh

class EmotionalMeshDetection:
    def __init__(self, static=False, max_num_faces=1, refine=False):
        # Initializing FaceMesh variables
        self.mp_face_mesh = mp.solutions.face_mesh
        self.face_mesh = self.mp_face_mesh.FaceMesh(
            static_image_mode=static,
            max_num_faces=max_num_faces,
            refine_landmarks=refine,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5)

        # Detected faces (list of EmotionalMesh())
        self.emotional_meshes = []
    
    def process(self, frame):
        self.emotional_meshes = []
        results = self.face_mesh.process(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        if results.multi_face_landmarks:
            for face_landmarks in results.multi_face_landmarks:
                self.emotional_meshes.append(EmotionalMesh(frame.shape))
                self.emotional_meshes[-1].update_landmarks(face_landmarks)

    def get_emotional_meshes(self):
        return self.emotional_meshes