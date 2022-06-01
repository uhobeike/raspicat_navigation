#!/bin/bash -e

trap "mv $1/map.stl $1/$2.stl && trap SIGINT" SIGINT

sleep 'infinity'