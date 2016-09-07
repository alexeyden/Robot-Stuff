import numpy as np
import cv2
import cv2.xfeatures2d
from matplotlib import pyplot as plt
import sys
import scipy.cluster.vq 

# изображение-образец
img1 = cv2.imread(sys.argv[1], 0)
# изображение для сравнения
img2 = cv2.imread(sys.argv[2], 0) 

# нормализация яркости
img1 = cv2.equalizeHist(img1)
img2 = cv2.equalizeHist(img2)

# создание SURF-детектора
surf = cv2.xfeatures2d.SURF_create()

# поиск ключевых точек и вычисление дескрипторов
kp1, des1 = surf.detectAndCompute(img1,None)
kp2, des2 = surf.detectAndCompute(img2,None)

# сопоставление дескрипторов изображений
flann = cv2.BFMatcher(cv2.NORM_L2)

matches = flann.knnMatch(des1,des2, k=2)

matchesMask = [[0,0] for i in range(len(matches))]
h,w = img1.shape
good = []

# отброс точек, находящихся на слишком большом расстоянии
for i,(m,n) in enumerate(matches):
	if m.distance < 0.8*n.distance:
		matchesMask[i] = [1,0]
		good.append(m)

# массивы точек для рассчета гомографии
src_pts = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)

M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)

# расчет гомографии
mm = mask.ravel().tolist()

j = 0
for i in range(0, len(matchesMask)):
	if matchesMask[i][0] == 1:
		if mm[j] == 0:
			matchesMask[i][0] = 0
		j += 1

# исходный прямоугольник изображения-образца
pts = np.float32([ [2,2],[0,h-3],[w-3,h-3],[w-3,0] ]).reshape(-1,1,2)

# преобразование прямоугольника
dst = cv2.perspectiveTransform(pts, M)

# отрисовка полученного многоугольника
img2 = cv2.polylines(img2,[np.int32(dst)],True,255,3, cv2.LINE_AA)

H = max(dst[1][0][1],dst[2][0][1]) - min(dst[0][0][1],dst[3][0][1])
W = max(dst[2][0][0],dst[3][0][0]) - min(dst[1][0][0],dst[0][0][0])
X1 = [dst[0][0][0], dst[2][0][0]]
Y1 = [dst[0][0][1], dst[2][0][1]]
X2 = [dst[1][0][0], dst[3][0][0]]
Y2 = [dst[1][0][1], dst[3][0][1]]

p1 = dst[0][0] 
p2 = dst[1][0]
p3 = dst[2][0]
p4 = dst[3][0]
norm = np.linalg.norm 

# проверка на искажения
a1 = np.arccos(np.dot((p1 - p2)/norm(p1 - p2), (p3 - p2)/norm(p3 - p2)))
a2 = np.arccos(np.dot((p4 - p3)/norm(p4 - p3), (p2 - p3)/norm(p2 - p3)))
a3 = np.arccos(np.dot((p1 - p4)/norm(p1 - p4), (p3 - p4)/norm(p3 - p4)))
a4 = np.arccos(np.dot((p4 - p1)/norm(p4 - p1), (p2 - p1)/norm(p2 - p1)))
deg = [a1*180/np.pi,a2*180/np.pi,a3*180/np.pi,a4*180/np.pi] # углы многоугольника
TH = np.pi * 3/4
if a1 > TH or a2 > TH or a3 > TH or a3 > TH or abs(sum(deg)-360) > 10:
	print('Несовпадние: ', deg, sum(deg))
else:
	print('Совпадение: ', deg, sum(deg))
	
draw_params = dict(matchColor = (0,255,0),
                   singlePointColor = (255,0,0),
                   matchesMask = matchesMask,
                   flags = 0)
# отрисовка совпадений
img3 = cv2.drawMatchesKnn(img1,kp1,img2,kp2,matches,None,**draw_params)

plt.imshow(img3,),plt.show()

