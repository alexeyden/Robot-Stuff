import os
import glob
import re
import numpy as np
import scipy.cluster.vq
import cv2
import cv2.xfeatures2d

from sklearn.svm import SVC
from sklearn.preprocessing import StandardScaler

from typing import List, Dict


class TrainingOptions:
    """
    Параметры обучения: опции SURF детектора и K-Means кластеризатора
    """
    def __init__(self, surf: dict, kmeans: dict, svm: dict):
        self.surf = surf
        self.kmeans = kmeans
        self.svm = svm


class ClassTrainingResult:
    """
    Результат бинарного классификатора
    """
    def __init__(self, vocabulary, svc: SVC, scaler: StandardScaler):
        self.vocabulary = vocabulary
        self.svc = svc
        self.scaler = scaler


class TrainingResult:
    """
    Результаты классификации для всех классов и параметры
    """
    def __init__(self, results: Dict[str, ClassTrainingResult], opts: TrainingOptions):
        """
        :param results: результаты для каждого класса
        :param opts: параметры
        """
        self.results = results
        self.options = opts


class TrainingImage:
    """
    Входная картинка для обучения классификатора
    """

    NAME_REGEX = r'(p|n)([0-9]+)'

    def __init__(self, class_, pos, path, index, descriptors):
        """
        :param class_: класс (X)
        :param pos: положительный (на картинке X) или отрицательный образец (на картинке не X)
        :param path: путь до картинки
        :param index: номер картинки
        :param descriptors: извлеченные из картинки дескрипторы
        """
        self.class_ = class_
        self.positive = pos
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
        pos, index = re.match(TrainingImage.NAME_REGEX, name.replace('.png', '')).groups(0)
        class_ = os.path.split(os.path.dirname(path))[-1]

        return TrainingImage(class_, pos == 'p', path, int(index), descriptors)

    def __str__(self):
        return '{0} ({1}{2})'.format(self.path, "-+"[self.positive], self.class_)


class MultipleClassTrainer:
    """
    Обучает бинарные классификаторы для всех классов
    """
    def __init__(self, options: TrainingOptions, data_path: str, dont_touch=list()):
        self.options = options
        self.path = data_path
        self.dont_touch = dont_touch

    def train(self):
        class_names = [path for path in os.listdir(self.path) if os.path.isdir(os.path.join(self.path, path))]

        results = dict()

        for class_name in class_names:
            if class_name in self.dont_touch:
                print(f'Ignoring {class_name}')
                continue

            path = os.path.join(self.path, class_name)

            print('Training for class {0}:'.format(class_name))

            class_results = BagOfWordsClassTrainer(self.options, path).train()
            results[class_name] = class_results

        return TrainingResult(results, self.options)


class BagOfWordsClassTrainer:
    """
    Обучает бинарный классификатор на основе Bag of words для одного класса (X vs не X)
    """
    def __init__(self, options: TrainingOptions, data_path: str):
        self.options = options
        self.surf_detector = cv2.xfeatures2d.SURF_create(**self.options.surf)
        self.class_ = os.path.split(os.path.dirname(data_path))[-1]

        image_paths = [os.path.join(data_path, path)
                       for path in os.listdir(data_path)
                       if re.match(TrainingImage.NAME_REGEX, path.replace('.png', ''))]
        image_paths.sort()

        self.training_images: List[TrainingImage] = []

        for image_path in image_paths:
            image = TrainingImage.read(image_path, self.surf_detector)

            if image is not None:
                self.training_images.append(image)
                print('Image {0}: {1} descriptors found'.format(image, len(image.descriptors)))
            else:
                print('Warning: no descriptors found for {0}'.format(image_path))

    def train(self):
        print('Training SVM classifier..')

        descriptors = self._flatten_descriptors()
        vocabulary, _ = scipy.cluster.vq.kmeans(descriptors, **self.options.kmeans)

        histogram = np.zeros((len(self.training_images), len(vocabulary)), 'float32')

        for index, image in enumerate(self.training_images):
            words, _ = scipy.cluster.vq.vq(image.descriptors, vocabulary)

            for word in words:
                histogram[index][word] += 1

        scaler = StandardScaler().fit(histogram)
        histogram = scaler.transform(histogram)

        svm = SVC(probability=True, **self.options.svm)
        class_ids = [1 if image.positive else 0 for image in self.training_images]
        svm.fit(histogram, np.array(class_ids))

        return ClassTrainingResult(
            vocabulary=vocabulary,
            svc=svm,
            scaler=scaler
        )

    def _flatten_descriptors(self):
        return [descr for image in self.training_images for descr in image.descriptors]
