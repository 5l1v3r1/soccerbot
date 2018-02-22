#!/bin/bash

cd ~/soccerbot/rcsoccersim/utra_base/

pkill -f rcssmonitor
pkill -f rcssserver
rcssserver &
sleep 0.5
rcssmonitor &
sleep 0.5

echo "Starting both teams"
~/soccerbot/rcsoccersim/utra_base/start.sh &
sleep 10
~/soccerbot/rcsoccersim/utra_base/start.sh -t "UTRA"
