#!/bin/sh
DISPLAY=`echo $1 | sed -n 's/.*:\([0-9][0-9]*\).*/\1/p'`
PORT=`expr 5900 + $DISPLAY`
websockify 6081 localhost:$PORT