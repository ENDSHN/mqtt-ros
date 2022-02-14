#!/bin/bash
cd ..

for i in {1..20}
do
    export ROBOT_NAME="robot$i"

    docker-compose -f docker-compose-multiple.yml -p p$i up -d --force-recreate
done
