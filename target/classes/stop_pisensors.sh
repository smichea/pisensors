#!/bin/bash
pgrep -f pisensor
if [ "$?" -ne "0" ]; then
  echo "Process pisensor not found"
  exit 1
else
  pkill -f pisensor
  dt=`date '+%Y/%m/%d %H:%M:%S'`
  echo "$dt : stopped process" >> logs/server.log
fi
