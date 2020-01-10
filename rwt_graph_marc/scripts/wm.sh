#!/bin/sh
ARGS=`echo $* | sed 's/[ ]_[^ ]*//g'`
ratpoison $ARGS