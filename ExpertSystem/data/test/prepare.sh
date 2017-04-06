#!/bin/bash

MAX_WIDTH=900
for img in $(ls | sort | sed "/$(basename $0)/d"); do
	if [[ "$(stat -c '%F' $img)" == "directory" ]]; then continue; fi; 

	echo "Processing $img..."

	as_png="${img%.*}.png"

	convert -resize $MAX_WIDTH\> $img $img
	convert -colorspace Gray $img $img

	if [[ "$img" != "$as_png" ]]; then
	    convert $img $as_png
	    rm $img
	fi
done
