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
	def __init__(self, data_path, dump_path = None, detector=('surf', {}), cluster=('k-means', {'k_or_guess': 20}), classify=('svm', {})):
		self.path = data_path
		self.detector = self._create_detector(detector)
		self.clusterizer = self._create_clusterizer(cluster)
#		self.classifier = #self._create_classifier(classify)
		self.cluster_k = cluster[1]['k_or_guess']

		names = os.listdir(self.path)
		self.data = {
				name: [
					self.path + '/' + name + '/' + image 
					for image in os.listdir(self.path + '/' + name)
				] for name in names
		}

		if dump_path is not None:
			self.result = joblib.load(dump_path)
		
	def train(self):
		image_list = [im for group in self.data.values() for im in group]

		descriptor_list = []
		
		for im_path in image_list:
			im_data = cv2.imread(im_path)
			key_points = self.detector.detect(im_data)
			key_points, descriptors = self.detector.compute(im_data, key_points)
			descriptor_list.append((im_path, descriptors)) 
		
		descriptors = np.array([descr for _, descr_list in descriptor_list for descr in descr_list])
		voc, variance = self.clusterizer(descriptors)
		
		im_features = np.zeros((len(image_list), self.cluster_k), 'float32')
		for i in range(len(image_list)):
			words, distance = scipy.cluster.vq.vq(descriptor_list[i][1], voc)
			
			for word in words:
				im_features[i][word] += 1
		
		nbr_occurences = np.sum( (im_features > 0) * 1, axis = 0)
		idf = np.array(np.log((1.0*len(image_list)+1) / (1.0*nbr_occurences + 1)), 'float32')
		scaler = StandardScaler().fit(im_features)
		im_features = scaler.transform(im_features)
		
		plt.imshow(im_features, cmap='gist_gray', interpolation='nearest')
		plt.xlabel(u'Номер слова (кластера)')
		plt.ylabel(u'Изображение')
		plt.colorbar()
		plt.yticks([0,1,2,3,4,5,6,7,8,9], ['car/1.jpg', 'car/2.jpg', 'car/3.jpg', 'car/4.jpg', 'car/5.jpg', 'building/1.jpg', 'building/2.jpg', 'building/3.jpg', 'building/4.jpg', 'building/5.jpg'])
		plt.show()
		sys.exit(0)
		
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
	
	def save(self, out_path):	
		joblib.dump(self.result, out_path, compress=3)

	def classify(self, img_path):
		img_data = cv2.imread(img_path)
		key_points = self.detector.detect(img_data)
		key_points, descriptors = self.detector.compute(img_data, key_points)
		features = np.zeros((1, self.result['cluster_k']), "float32")
		words, distance = scipy.cluster.vq.vq(descriptors, self.result['voc'])
		
		for word in words:
			features[0][word] += 1
		
		nbr_occurences = np.sum( (features > 0) * 1, axis = 0)
		idf = np.array(np.log((2.0) / (1.0*nbr_occurences + 1)), 'float32')
		
		features = self.result['scaler'].transform(features)
		
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

if sys.argv[1] == 'train':		
	BOW(data_path='data').train().save(sys.argv[2])
elif sys.argv[1] == 'class':
	pred = BOW(data_path='data', dump_path = sys.argv[2]).classify(sys.argv[3])
	print(pred)
