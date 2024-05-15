#!/bin/bash
echo "Launching MH01 with Stereo sensor"
./final_vo ./Vocabulary/ORBvoc.txt ./Settings/EuRoC.yaml /home/ali/Downloads/MH_01_easy ./Settings/EuRoC_TimeStamps/MH01.txt dataset-MH01_stereo
