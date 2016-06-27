#!/usr/bin/env python3

import numpy as np
import cv2
import cv2.xfeatures2d as xf2d

cap = cv2.VideoCapture(0)
surf = xf2d.SURF_create(400)

while True:
	ret, frame = cap.read()
	
	gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
	
	kp, des = surf.detectAndCompute(gray, None)
	cv2.drawKeypoints(frame, kp, frame, (255, 0, 0), 4)

	cv2.putText(frame, str(len(kp)), (20, 20), cv2.FONT_HERSHEY_PLAIN, 1, (255, 255, 0))
	cv2.putText(frame, str(surf.getHessianThreshold()), (20, 40), cv2.FONT_HERSHEY_PLAIN, 1, (255, 255, 0))

	cv2.imshow('frame', frame)
	
	key = cv2.waitKey(1) & 0xFF 
	if key == ord('e'):
		surf.setHessianThreshold(surf.getHessianThreshold() + 100)
	elif key == ord('w'):
		surf.setHessianThreshold(surf.getHessianThreshold() - 100)
	elif key == ord('q'):
		break

cap.release()
cv2.destroyAllWindows()
