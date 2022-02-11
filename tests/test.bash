#!/bin/bash
cd ..

for i in {1..20}
do
    export ROBOT_NAME="robot$i"

    docker-compose -p p$i up -d
done
