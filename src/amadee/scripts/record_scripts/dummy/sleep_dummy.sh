#!/bin/bash

# trap break SIGINT

for (( c=0; c<=100; c++ ))
do
    # sleep potentially for a long time
    echo "Iteration ${c} with arg $1"
    sleep 2
done
