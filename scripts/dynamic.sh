#!/bin/bash

TUM_PATH=/home/jarvis/hdd/Dataset/SLAMDataset/tum
Dataset=$1
path=$TUM_PATH/$Dataset
expath=/home/jarvis/jw_ws/edge_cloud/optical_flow

mkdir $path/optical_flow
for ((i=2; i<=20; i+=2));
do
    echo "Sparse Optical Flow Threshold is $i"
    foo=$(printf "%02d" $i)
    echo $foo
    mkdir $path/optical_flow/result_dynamic_$foo
    cd $expath
    ./main -p=$path -n=$i -o=$path/optical_flow/result_dynamic_$foo
done