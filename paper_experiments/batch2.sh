#!/usr/bin/env bash

NN=noisy
FT=0
REP=50
ST=0

mkdir -p jobs ${NN}
pushd ${NN}
mkdir -p poses
mkdir -p com
mkdir -p springs
mkdir -p angles
popd

for SL in 1 7; do
for RS in $(seq 1 $REP); do
    EXPID=${RS}_${SL}_${ST}_${FT}_${NN}
    rm -f "${NN}/com/com_${EXPID}.tsv"
done
done

for SL in 1 7; do
for RS in $(seq 50 100); do
    EXPID=${RS}_${SL}_${ST}_${FT}_${NN}
    if [[ ! -f "${NN}/com/com_${EXPID}.tsv" ]]; then
        while ! sbatch -J $(basename $0) ./run_experiment.sh $RS $SL $ST $FT $NN; do
            sleep 5
        done
    fi
done
done
