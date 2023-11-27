#!/bin/bash

cd /home/pi/piPaw/
until python3 -u piPaw_amplitude.py &>>log.txt; do
	echo "'piPaw program' crashed with exit code $?. Restarting..." >&2
	sleep 1
done
