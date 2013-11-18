#!/usr/local/bin/bash

if [ $# -ne 3 ]
then
	echo "Usage: run.sh input_cell output_cell design_rule"
	exit
else
	strm2oa -lib DesignLib -gds ../testcases/$1.gds -layerMap ./layer.map
	cp ../testcases/$1.txt ./
	./main $1 $2 $1.txt $3
	oa2strm -lib DesignLib -cell $2 -gds new.gds -layerMap ./layer.map
fi
