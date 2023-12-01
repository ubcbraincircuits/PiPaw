#!/bin/bash

cd /home/pi/PiPaw/
until python3 -u src/piPaw_amplitude.py &>>log.txt; do
	echo "'piPaw program' crashed with exit code $?. Restarting..." >&2
	sleep 1
done
