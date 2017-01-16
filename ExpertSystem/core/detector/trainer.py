import os
import cv2
import cv2.xfeatures2d
import numpy as np
import glob
import re

from sklearn.svm import LinearSVC
import scipy.cluster.vq
from sklearn.preprocessing import StandardScaler


class TrainingImage:
    def __init__(self, class_, path, index, descriptors):
        self.class_ = class_
        self.path = path
        self.index = index
        self.descriptors = descriptors

    @staticmethod
    def read(path, detector):
        image = cv2.imread(path)

        key_points = detector.detect(image)
        _, descriptors = detector.compute(image, key_points)

        if descriptors is None:
            return None

        name = os.path.basename(path)
        class_, index = re.match('(.*)_([0-9])+', name.replace('.png', '')).groups(0)

        return TrainingImage(class_, path, int(index), descriptors)


class TrainingResult:
    def __init__(self, vocabulary, svc, options, classes, scaler):
        self.vocabulary = vocabulary
        self.svc = svc
        self.options = options
        self.classes = classes
        self.scaler = scaler


class TrainingOptions:
    def __init__(self, surf: dict, kmeans: dict):
        self.surf = surf
        self.kmeans = kmeans


class BagOfWordsTrainer:
    def __init__(self, options: TrainingOptions, data_path : str):
        self.options = options
        self.surf_detector = cv2.xfeatures2d.SURF_create(**self.options.surf)

        image_paths = glob.glob(os.path.join(data_path, '*_*.png'))

        self.training_images = []

        for image_path in image_paths:
            image = TrainingImage.read(image_path, self.surf_detector)

            if image is not None:
                self.training_images.append(image)
            else:
                print('Warning: no descriptors found for {0}'.format(image_path))

        classes = {
            image.class_
            for image in self.training_images
        }
        self.classes = {class_: id_ for id_, class_ in enumerate(classes)}

    def train(self):
        descriptors = self._flatten_descriptors()
        vocabulary, _ = scipy.cluster.vq.kmeans(descriptors, **self.options.kmeans)

        histogram = np.zeros((len(self.training_images), self.options.kmeans['k_or_guess']), 'float32')

        for index, image in enumerate(self.training_images):
            words, _ = scipy.cluster.vq.vq(image.descriptors, vocabulary)

            for word in words:
                histogram[index][word] += 1

        scaler = StandardScaler().fit(histogram)
        histogram = scaler.transform(histogram)

        svm = LinearSVC()
        class_ids = [self.classes[image.class_] for image in self.training_images]
        svm.fit(histogram, np.array(class_ids))

        return TrainingResult(
            vocabulary=vocabulary,
            svc=svm,
            options=self.options,
            classes=self.classes,
            scaler=scaler
        )

    def _flatten_descriptors(self):
        return [descr for image in self.training_images for descr in image.descriptors]
