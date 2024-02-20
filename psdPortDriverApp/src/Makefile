TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

LIBRARY_IOC += PSDPortDriverSupport
PSDPortDriverSupport_SRCS += psdPortDriver.cpp
PSDPortDriverSupport_LIBS += asyn
PSDPortDriverSupport_LIBS += $(EPICS_BASE_IOC_LIBS)

#=============================
# Build the IOC application

PROD_IOC = psdPortDriver
# psdPortDriver.dbd will be created and installed
DBD += psdPortDriver.dbd

# psdPortDriver.dbd will be made up from these files:
psdPortDriver_DBD += base.dbd

# Include dbd files from all support applications:
#psdPortDriver_DBD += xxx.dbd

# Add all the support libraries needed by this IOC
psdPortDriver_LIBS += PSDPortDriverSupport asyn

# psdPortDriver_registerRecordDeviceDriver.cpp derives from psdPortDriver.dbd
psdPortDriver_SRCS += psdPortDriver_registerRecordDeviceDriver.cpp

# Build the main IOC entry point on workstation OSs.
psdPortDriver_SRCS_DEFAULT += psdPortDriverMain.cpp
psdPortDriver_SRCS_vxWorks += -nil-

# Add support from base/src/vxWorks if needed
#psdPortDriver_OBJS_vxWorks += $(EPICS_BASE_BIN)/vxComLibrary

# Finally link to the EPICS Base libraries
psdPortDriver_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

CODE_CXXFLAGS +=  -Wimplicit-fallthrough

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

