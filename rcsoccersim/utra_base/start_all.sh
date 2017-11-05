#!/bin/sh

echo "Starting both teams"
sh start.sh &
sleep 20
sh start.sh -t "UTRA"
