#!/bin/bash

DATASET=/home/jarvis/jw_ws/edge_cloud/optical_flow/dataset/rgbd_dataset_freiburg1_teddy

for ((i=2;i<=20;i+=2)); 
do
    echo "Optical Flow Threshold is $i"
    foo=$(printf "%02d" $i)
    echo $foo
    mkdir $DATASET/optical_flow/result_sparse_$foo
    ../cal_optical_flow -p=$DATASET -n=10 -o=$DATASET/optical_flow/result_sparse_$foo
done
