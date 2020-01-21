#!/bin/sh
DISPLAY=`echo $1 | sed -n 's/.*:\([0-9][0-9]*\).*/\1/p'`
Xvnc -SecurityTypes None -depth 24 -geometry 1280x768 :$DISPLAY