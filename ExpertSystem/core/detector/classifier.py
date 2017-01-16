import cv2.xfeatures2d

from .trainer import *


class BagOfWordsClassifier:
    def __init__(self, training: TrainingResult):
        self.training = training
        self.surf_detector = cv2.xfeatures2d.SURF_create(**training.options.surf)
        self.classes = {id_: class_ for class_, id_ in self.training.classes.items()}

    def predict(self, image_or_path):
        if type(image_or_path) is str:
            return self._predict_path(image_or_path)
        elif type(image_or_path) is np.ndarray:
            return self._predict_image(image_or_path)
        else:
            raise TypeError("Argument must be either ndarray or image file path")

    def _predict_image(self, image):
        key_points = self.surf_detector.detect(image)
        key_points, descriptors = self.surf_detector.compute(image, key_points)
        features = np.zeros((1, self.training.options.kmeans['k_or_guess']), "float32")

        words, distance = scipy.cluster.vq.vq(descriptors, self.training.vocabulary)

        for word in words:
            features[0][word] += 1

        features = self.training.scaler.transform(features)

        predicted_class_id = self.training.svc.predict(features)[0]
        distances = { self.classes[id_]: dist for id_, dist in enumerate(self.training.svc.decision_function(features)[0])}
        prediction = (self.classes[predicted_class_id], distances)

        return prediction

    def _predict_path(self, path):
        image = cv2.imread(path)
        return self._predict_image(image)
