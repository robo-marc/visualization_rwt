#!/bin/sh
ARGS=`echo $* | sed 's/[ ]_[^ ]*//g'`
while true; do ratpoison $ARGS; done