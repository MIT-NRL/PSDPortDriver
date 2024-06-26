#!../../bin/linux-aarch64/psdPortDriver

< envPaths

cd "${TOP}"

## Register all support components
dbLoadDatabase "dbd/psdPortDriver.dbd"
psdPortDriver_registerRecordDeviceDriver pdbbase

## Setup PSD
# psdPortDriverConfigure("he3PSD", "127.0.0.1", "8765", "7654")
# psdPortDriverConfigure("he3PSD", "192.168.0.17", "23", "4660")
dbLoadRecords("db/psdPortDriver.template","P=he3PSD:,R=det1:,PORT=he3PSD,ADDR=0,TIMEOUT=1,NBINS=4")

## List available database records
dbl

cd "${TOP}/iocBoot/${IOC}"
iocInit
