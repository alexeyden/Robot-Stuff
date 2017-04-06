#!/usr/bin/python3

from build.ffld_detector import PyFFLDDetector
from scipy.misc import imread
import numpy as np
import sys
import cv2
from datetime import datetime as dt

f = PyFFLDDetector(sys.argv[1])
f.threshold = -0.5
f.interval = 4

cap = cv2.VideoCapture(0)

while True:
	t0 = dt.now()

	ret, frame = cap.read()
	dts = f.detect(frame)

	t = dt.now() - t0
	msec = t.microseconds	/ 1000.0

	cv2.putText(frame, f'dt = {msec:.0f} ms', (0, 20), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 0))

	for d in dts:
		score, x0, y0, x1, y1 = d

		cv2.putText(frame, f'{score:.2f}', (x0, y0 - 20), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 0))
		cv2.rectangle(frame, (x0, y0), (x1, y1), (0, 255, 100), 2)

	cv2.imshow('frame', frame)

	if cv2.waitKey(1) & 0xFF == ord('q'):
			break

cap.release()
cv2.destroyAllWindows()

