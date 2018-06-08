#!/bin/bash


prefix="augmented_training/"
file_paths=`ls *.jpg`

for file in $file_paths
do
    #echo $prefix$file >> ./distance-test.list
    echo $file >> ./distance-test.list
done
