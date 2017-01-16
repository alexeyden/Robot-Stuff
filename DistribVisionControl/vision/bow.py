#!/bin/env python3

import os
import sys
import cv2
import cv2.xfeatures2d
import numpy as np

from sklearn.svm import LinearSVC
from sklearn.externals import joblib
import scipy.cluster.vq 
from sklearn.preprocessing import StandardScaler
import matplotlib.pyplot as plt
from matplotlib import rc

font = {'family' : 'DejaVu Sans',
        'size'   : 22}

rc('font', **font)

class BOW:
	# инициализация
	def __init__(self, data_path, dump_path = None, detector=('surf', {}), cluster=('k-means', {'k_or_guess': 20}), classify=('svm', {})):
		self.path = data_path # путь до тренировочных изображений
		self.detector = self._create_detector(detector) # тип детектора
		self.clusterizer = self._create_clusterizer(cluster) # тип кластеризатора
		self.cluster_k = cluster[1]['k_or_guess'] # размер словаря

		# чтение путей изображений
		if data_path is not None:
			names = os.listdir(self.path)
			self.data = {
					name: [
						self.path + '/' + name + '/' + image 
						for image in os.listdir(self.path + '/' + name)
					] for name in names
			}
		
		# чтение обученной базы данных
		if dump_path is not None:
			self.result = joblib.load(dump_path)
		
	# выполнить обучение базы
	def train(self):
		image_list = [im for group in self.data.values() for im in group]

		descriptor_list = []
		
		# чтение изображений и выделение дескрипторов
		for im_path in image_list:
			im_data = cv2.imread(im_path)
			key_points = self.detector.detect(im_data)
			key_points, descriptors = self.detector.compute(im_data, key_points)
			descriptor_list.append((im_path, descriptors)) 
		
		# кластеризация и получение словаря
		descriptors = np.array([descr for _, descr_list in descriptor_list for descr in descr_list])
		voc, variance = self.clusterizer(descriptors)
		
		# квантование векторов и построение гистограммы
		im_features = np.zeros((len(image_list), self.cluster_k), 'float32')
		for i in range(len(image_list)):
			words, distance = scipy.cluster.vq.vq(descriptor_list[i][1], voc)
			
			for word in words:
				im_features[i][word] += 1
		
		# нормировка гистограммы
		scaler = StandardScaler().fit(im_features)
		im_features = scaler.transform(im_features)
		
		# обучение классификатора
		clf = LinearSVC()
		class_ids = [class_id for class_id,images in enumerate(self.data.values()) for _ in images]
		clf.fit(im_features, np.array(class_ids))	

		self.result = {
			'class_result' : clf,
			'tags' : list(self.data.keys()),
			'scaler' : scaler,
			'cluster_k' : self.cluster_k,
			'voc' : voc
		}
		
		return self
	
	# сохранить обученную базу
	def save(self, out_path):	
		joblib.dump(self.result, out_path, compress=3)

	# классификация изображения
	def classify(self, img_path):
		# чтение и выделение дескрипторов
		img_data = cv2.imread(img_path)
		key_points = self.detector.detect(img_data)
		key_points, descriptors = self.detector.compute(img_data, key_points)
		features = np.zeros((1, self.result['cluster_k']), "float32")
		# квантование по готовому словарю
		words, distance = scipy.cluster.vq.vq(descriptors, self.result['voc'])
		
		# построение нистограммы
		for word in words:
			features[0][word] += 1
		
		# нормализация
		features = self.result['scaler'].transform(features)
		
		# определение класса обученным классификатором
		prediction = [self.result['tags'][i] for i in self.result['class_result'].predict(features)]
		return prediction
		
	def _create_clusterizer(self, type_):
		if type_[0] == 'k-means':
			return lambda descr: scipy.cluster.vq.kmeans(descr, **type_[1]) 
		else:
			return None

	def _create_detector(self, type_):
		if type_[0] == 'surf':
			return cv2.xfeatures2d.SURF_create(**type_[1])
		else:
			return None

# запуск для обучения или классификации
if sys.argv[1] == 'train':
	BOW(data_path='data').train().save(sys.argv[2])
elif sys.argv[1] == 'class':
	images = []
	
	if os.path.isfile(sys.argv[3]):
		images += [sys.argv[3]]
	else:
		images = [os.path.join(sys.argv[3], img_file) for img_file in os.listdir(sys.argv[3]) if not img_file.startswith('.')]
	
	print('<table border=1>')
	for image in sorted(images):
		pred = BOW(data_path=None, dump_path = sys.argv[2]).classify(image)
		print('<tr><td><img width=200 src="' + image + '"/></td><td>', image, '</td><td>', pred[0], '</td></tr>')
	print('</table>')

