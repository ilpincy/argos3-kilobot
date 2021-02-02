#!/usr/bin/env bash
#SBATCH -N 1
#SBATCH -n 1
#SBATCH --mem 1Gb
#SBATCH -p short
#SBATCH -t 24:00:00
#SBATCH -C E5-2695

#
# CHECK PARAMS
#
echo "ARGS = $@"
if [[ $# -ne 5 ]]; then
    echo "$0 <RANDOMSEED> <ROBOTSxSIDE> <STIFFNESS> <FAULTY> <NOISE>"
    exit 1
fi

#
# PARAMETERS
#
RANDOMSEED=$1
ROBOTSxSIDE=$2
STIFFNESS=$3
FAULTY=$4
NOISE=$5
TPS=32

#
# FOLDERS
#
DATADIR=$PWD
EXPID=${RANDOMSEED}_${ROBOTSxSIDE}_${STIFFNESS}_${FAULTY}_${NOISE}
TMPDIR=${DATADIR}/jobs
EXPDIR=${TMPDIR}/kbsr_${EXPID}
EXPFILE=exp_${EXPID}.argos

#
# Avoid duplicates
#
if [[ -f "${DATADIR}/${NOISE}/com/com_${EXPID}.tsv" ]]; then
        echo "${EXPID} already run"
        exit 0
fi

#
# CREATE WORKSPACE
#
rm -rf ${EXPDIR}
mkdir -p ${EXPDIR}
cd ${EXPDIR}
sed -e "s|__RANDOMSEED__|${RANDOMSEED}|"   \
    -e "s|__TPS__|${TPS}|"                 \
    -e "s|__DATADIR__|${DATADIR}|g"        \
    -e "s|__EXPID__|${EXPID}|"             \
    -e "s|__ROBOTSxSIDE__|${ROBOTSxSIDE}|" \
    -e "s|__STIFFNESS__|${STIFFNESS}|"     \
    -e "s|__FAULTY__|${FAULTY}|"           \
    ${DATADIR}/${NOISE}.argos              \
    > ${EXPFILE}

#
# RUN EXPERIMENT
#
argos3 -n -e /dev/null -l /dev/null -c ${EXPFILE}

#
# COPY DATA
#
for D in poses com springs angles; do
    mv ${D}_${EXPID}.tsv ${DATADIR}/${NOISE}/${D}
done

#
# CLEANUP
#
cd ${DATADIR}
rm -rf ${EXPDIR}
