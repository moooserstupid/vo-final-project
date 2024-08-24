#!/bin/bash
echo "Launching Custom Dataset with Stereo sensor"
./final_vo ./Vocabulary/ORBvoc.txt ./Settings/Custom_Ali.yaml /home/ali/Work/Programming/Cpp/calibration/dataset-ground /home/ali/Work/Programming/Cpp/calibration/dataset-ground/custom_timestamps.txt
