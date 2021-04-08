#!/usr/bin/env bash

NN=$1
ST=$2
SL=7

mkdir -p ${NN}
pushd ${NN}
mkdir -p poses
mkdir -p com
mkdir -p springs
mkdir -p angles
popd

for REP in `seq 1 10`; do
    for FS in 0 1 2 3 4 5; do
        if [[ ${FS} -lt $(( ${SL} * ${SL} )) ]]; then
            echo
            ./run_experiment.sh ${REP} ${SL} ${ST} ${FS} ${NN}
        fi
    done
done
