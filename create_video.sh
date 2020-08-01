#!/bin/sh

ffmpeg -r 1 -f image2 -s 1920x1080 -start_number 1 -i test%08d.ppm -vframes 180 -vcodec libx264 -crf 25  -pix_fmt yuv420p test.mp4