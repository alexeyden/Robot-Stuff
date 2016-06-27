#!/usr/bin/env python3

import numpy as np
import cv2
import cv2.xfeatures2d as xf2d
import sys

cap = cv2.VideoCapture(0)
surf = xf2d.SURF_create()

while True:
	frame = cv2.imread(sys.argv[1])
	gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

	kp, des = surf.detectAndCompute(gray, None)
	cv2.drawKeypoints(frame, kp, frame, (0, 0, 0), 4)

	cv2.imshow('frame', frame)
	
	key = cv2.waitKey(1) & 0xFF 
	if key == ord('e'):
		surf.setHessianThreshold(surf.getHessianThreshold() + 100)
	elif key == ord('w'):
		surf.setHessianThreshold(surf.getHessianThreshold() - 100)
	elif key == ord('q'):
		break
	elif key == ord('s'):
		for i,d in enumerate(des):
			print(kp[i].pt, d)
		cv2.imwrite('out.png', frame)
		print(len(des))

cap.release()
cv2.destroyAllWindows()
