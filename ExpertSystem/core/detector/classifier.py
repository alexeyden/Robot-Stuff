import cv2.xfeatures2d

from .trainer import *


class BagOfWordsClassifier:
    def __init__(self, training: ClassTrainingResult, options: TrainingOptions):
        self.training = training
        self.surf_detector = cv2.xfeatures2d.SURF_create(**options.surf)

    def predict(self, image_or_path):
        if type(image_or_path) is str:
            return self._predict_path(image_or_path)
        elif type(image_or_path) is np.ndarray:
            return self._predict_image(image_or_path)
        else:
            raise TypeError("Argument must be either ndarray or image file path")

    def _predict_image(self, image):
        key_points = self.surf_detector.detect(image)

        if not key_points:
            return None

        key_points, descriptors = self.surf_detector.compute(image, key_points)
        features = np.zeros((1, len(self.training.vocabulary)), np.float32)

        words, distance = scipy.cluster.vq.vq(descriptors, self.training.vocabulary)

        for word in words:
            features[0][word] += 1

        features = self.training.scaler.transform(features)

        prob = self.training.svc.predict_proba(features)[0]

        return prob[1]

    def _predict_path(self, path):
        image = cv2.imread(path)
        return self._predict_image(image)
