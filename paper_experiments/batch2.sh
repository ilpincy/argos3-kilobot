#!/usr/bin/env bash

SL=7
NN=noisy
FT=0
REP=30
ST=0

mkdir -p jobs ${NN}
pushd ${NN}
mkdir -p poses
mkdir -p com
mkdir -p springs
mkdir -p angles
popd

for RS in $(seq 1 $REP); do
    EXPID=${RS}_${SL}_${ST}_${FT}_${NN}
    if [[ ! -f "${NN}/com/com_${EXPID}.tsv" ]]; then
        while ! sbatch -J $(basename $0) ./run_experiment.sh $RS $SL $ST $FT $NN; do
            sleep 5
        done
    fi
done
