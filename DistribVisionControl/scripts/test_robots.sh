#!/bin/bash

./mockupctl.py robot-connect robot-1 -long 3 -lat 3
./mockupctl.py robot-connect robot-2 -long 3 -lat 9
./mockupctl.py robot-connect robot-3 -long 8 -lat 6
./mockupctl.py robot-connect robot-4 -long 12 -lat 13
./mockupctl.py robot-connect robot-5 -long 18 -lat 10

./mockupctl.py robot-domain robot-1 -domain '["robot-2", "robot-3", "robot-4", "robot-5"]'
./mockupctl.py robot-domain robot-2 -domain '["robot-3", "robot-4", "robot-5"]'
./mockupctl.py robot-domain robot-3 -domain '["robot-4", "robot-5"]'
./mockupctl.py robot-domain robot-4 -domain '["robot-5"]'

exit 0

./mockupctl.py robot-goto robot-1 long=14.75 lat=2.8
./mockupctl.py robot-goto robot-2 long=13.5 lat=5.4
./mockupctl.py robot-goto robot-3 long=15.7 lat=7.67 &&

sleep 15s

./mockupctl.py robot-cam-turn robot-1 angle=15
./mockupctl.py robot-cam-turn robot-2 angle=33
./mockupctl.py robot-cam-turn robot-3 angle=3 &&

sleep 0.5s

./mockupctl.py robot-cam-turn robot-1 angle=30
./mockupctl.py robot-cam-turn robot-2 angle=66
./mockupctl.py robot-cam-turn robot-3 angle=3 &&

sleep 0.5s

./mockupctl.py robot-cam-turn robot-1 angle=45
./mockupctl.py robot-cam-turn robot-2 angle=100
./mockupctl.py robot-cam-turn robot-3 angle=10

