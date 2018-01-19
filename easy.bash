source /reg/g/pcds/setup/epicsenv-3.14.12.sh

#make clean
make

cd iocs/visarCameraIOC/iocBoot/iocVisarCamera/
./run
