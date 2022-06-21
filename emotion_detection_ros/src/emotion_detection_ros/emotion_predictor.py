import rospkg
import os
import numpy as np
import pickle

from emotion_detection_ros.emotional_mesh_detection import EmotionalMeshDetection
from emotion_detection_ros.emotion import Emotion

class EmotionPredictor:
    def __init__(self, model_algorithm="MLP", max_num_faces=1):
        # Emotional mesh to extract angles data
        self.emotional_mesh_detection = EmotionalMeshDetection(max_num_faces=max_num_faces)

        # Model and PCA to predict
        self.model = None
        self.pca = None
        self.model_algorithm = model_algorithm
        self.__initialize_model_and_pca(model_algorithm)

    def predict(self, frame):
        emotions = []
        self.emotional_mesh_detection.process(frame)
        emotional_meshes = self.emotional_mesh_detection.get_emotional_meshes()
        for e_mesh in emotional_meshes:
            # Prediction with angles of emotional mesh
            angles = e_mesh.angles
            angles = self.pca.transform(angles)
            if self.model_algorithm == "SVM":
                emotion_index = self.model.predict(angles)
                predicted_emotion = self.__evaluate_prediction(emotion_index)
                probability = 0
            else:
                probabilities = self.model.predict_proba(angles)
                emotion_index = np.argmax(probabilities)
                predicted_emotion = self.__evaluate_prediction_proba(emotion_index)
                probability = probabilities[0][emotion_index]

            # Save data in object Face()
            emotions.append(Emotion())
            emotions[-1].class_name = predicted_emotion
            emotions[-1].probability = round(probability*100, 2)
            if self.model_algorithm == "SVM":
                emotions[-1].label = emotions[-1].class_name
            else:
                emotions[-1].label = "{}: {} %".format(emotions[-1].class_name, emotions[-1].probability)
            emotions[-1].bounding_box = e_mesh.bounding_box
        return emotions

    # Private methods ----------------------------------------------------------------------
    def __initialize_model_and_pca(self, model_algorithm):
        model_file = os.path.join(
            rospkg.RosPack().get_path("emotion_detection_ros"),
            "models",
            "model_"+model_algorithm+".pkl",
        )
        with open(model_file, 'rb') as modelfile:
            loaded = pickle.load(modelfile)
        self.model = loaded['model']
        self.pca = loaded['pca_fit']

    def __evaluate_prediction_proba(self, emotion_index):
        if emotion_index == 0:
            return "Anger"
        elif emotion_index == 1:
            return "Happy"
        elif emotion_index == 2:
            return "Sadness"
        elif emotion_index == 3:
            return "Surprise"
        return None

    def __evaluate_prediction(self, emotion_index):
        if emotion_index == 1:
            return "Anger"
        elif emotion_index == 5:
            return "Happy"
        elif emotion_index == 6:
            return "Sadness"
        elif emotion_index == 7:
            return "Surprise"
        return None