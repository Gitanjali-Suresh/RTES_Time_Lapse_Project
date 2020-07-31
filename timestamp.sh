#!/bin/sh

datetime=`date`
INPUT_FILE=$1
machine=$2
platform=$3
convert ${INPUT_FILE} -pointsize 17 -fill white -undercolor '#00000080' -gravity SouthEast -annotate +0+5 "$datetime" ${INPUT_FILE}
convert ${INPUT_FILE} -pointsize 17 -fill white -undercolor '#00000080' -gravity SouthEast -annotate +0+25 "$machine $platform" ${INPUT_FILE}