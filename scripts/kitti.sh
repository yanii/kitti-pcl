#!/bin/bash
for file in *.bin
do 
	kitti2pcd --infile $file --outfile `basename $file .bin`.pcd
	mkdir `basename $file .bin`; kittitrackletextract --infile `basename $file .bin`.pcd --outfile `basename $file .bin`/`basename $file .bin`_ --tracklets ../../tracklet_labels.xml --frameid `basename $file .bin`
done
