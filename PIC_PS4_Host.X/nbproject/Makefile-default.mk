#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Include project Makefile
ifeq "${IGNORE_LOCAL}" "TRUE"
# do not include local makefile. User is passing all local related variables already
else
include Makefile
# Include makefile containing local settings
ifeq "$(wildcard nbproject/Makefile-local-default.mk)" "nbproject/Makefile-local-default.mk"
include nbproject/Makefile-local-default.mk
endif
endif

# Environment
MKDIR=gnumkdir -p
RM=rm -f 
MV=mv 
CP=cp 

# Macros
CND_CONF=default
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
IMAGE_TYPE=debug
OUTPUT_SUFFIX=elf
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/PIC_PS4_Host.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=hex
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/PIC_PS4_Host.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
endif

ifeq ($(COMPARE_BUILD), true)
COMPARISON_BUILD=
else
COMPARISON_BUILD=
endif

ifdef SUB_IMAGE_ADDRESS

else
SUB_IMAGE_ADDRESS_COMMAND=
endif

# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}

# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Source Files Quoted if spaced
SOURCEFILES_QUOTED_IF_SPACED=../Common/uart2.c "../DEE Emulation 16-bit/DEE Emulation 16-bit.c" "../DEE Emulation 16-bit/Flash Operations.s" ../USB/usb_host.c ../USB/usb_host_generic_PS4.c ../main.c ../usb_config.c ../outputs.c ../functional_test.c ../PS4_SPI.c ../PS4_controller.c

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/_ext/2108356922/uart2.o "${OBJECTDIR}/_ext/869223802/DEE Emulation 16-bit.o" "${OBJECTDIR}/_ext/869223802/Flash Operations.o" ${OBJECTDIR}/_ext/1360907413/usb_host.o ${OBJECTDIR}/_ext/1360907413/usb_host_generic_PS4.o ${OBJECTDIR}/_ext/1472/main.o ${OBJECTDIR}/_ext/1472/usb_config.o ${OBJECTDIR}/_ext/1472/outputs.o ${OBJECTDIR}/_ext/1472/functional_test.o ${OBJECTDIR}/_ext/1472/PS4_SPI.o ${OBJECTDIR}/_ext/1472/PS4_controller.o
POSSIBLE_DEPFILES=${OBJECTDIR}/_ext/2108356922/uart2.o.d "${OBJECTDIR}/_ext/869223802/DEE Emulation 16-bit.o.d" "${OBJECTDIR}/_ext/869223802/Flash Operations.o.d" ${OBJECTDIR}/_ext/1360907413/usb_host.o.d ${OBJECTDIR}/_ext/1360907413/usb_host_generic_PS4.o.d ${OBJECTDIR}/_ext/1472/main.o.d ${OBJECTDIR}/_ext/1472/usb_config.o.d ${OBJECTDIR}/_ext/1472/outputs.o.d ${OBJECTDIR}/_ext/1472/functional_test.o.d ${OBJECTDIR}/_ext/1472/PS4_SPI.o.d ${OBJECTDIR}/_ext/1472/PS4_controller.o.d

# Object Files
OBJECTFILES=${OBJECTDIR}/_ext/2108356922/uart2.o ${OBJECTDIR}/_ext/869223802/DEE\ Emulation\ 16-bit.o ${OBJECTDIR}/_ext/869223802/Flash\ Operations.o ${OBJECTDIR}/_ext/1360907413/usb_host.o ${OBJECTDIR}/_ext/1360907413/usb_host_generic_PS4.o ${OBJECTDIR}/_ext/1472/main.o ${OBJECTDIR}/_ext/1472/usb_config.o ${OBJECTDIR}/_ext/1472/outputs.o ${OBJECTDIR}/_ext/1472/functional_test.o ${OBJECTDIR}/_ext/1472/PS4_SPI.o ${OBJECTDIR}/_ext/1472/PS4_controller.o

# Source Files
SOURCEFILES=../Common/uart2.c ../DEE Emulation 16-bit/DEE Emulation 16-bit.c ../DEE Emulation 16-bit/Flash Operations.s ../USB/usb_host.c ../USB/usb_host_generic_PS4.c ../main.c ../usb_config.c ../outputs.c ../functional_test.c ../PS4_SPI.c ../PS4_controller.c


CFLAGS=
ASFLAGS=
LDLIBSOPTIONS=

############# Tool locations ##########################################
# If you copy a project from one host to another, the path where the  #
# compiler is installed may be different.                             #
# If you open this project with MPLAB X in the new host, this         #
# makefile will be regenerated and the paths will be corrected.       #
#######################################################################
# fixDeps replaces a bunch of sed/cat/printf statements that slow down the build
FIXDEPS=fixDeps

.build-conf:  ${BUILD_SUBPROJECTS}
ifneq ($(INFORMATION_MESSAGE), )
	@echo $(INFORMATION_MESSAGE)
endif
	${MAKE}  -f nbproject/Makefile-default.mk dist/${CND_CONF}/${IMAGE_TYPE}/PIC_PS4_Host.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

MP_PROCESSOR_OPTION=24FJ64GB106
MP_LINKER_FILE_OPTION=,-Tp24FJ64GB106.gld
# ------------------------------------------------------------------------------------
# Rules for buildStep: assemble
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/_ext/869223802/Flash\ Operations.o: ../DEE\ Emulation\ 16-bit/Flash\ Operations.s  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/869223802" 
	@${RM} "${OBJECTDIR}/_ext/869223802/Flash Operations.o".d 
	@${RM} "${OBJECTDIR}/_ext/869223802/Flash Operations.o".ok ${OBJECTDIR}/_ext/869223802/Flash\ Operations.o.err 
	@${RM} "${OBJECTDIR}/_ext/869223802/Flash Operations.o" 
	@${FIXDEPS} "${OBJECTDIR}/_ext/869223802/Flash Operations.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_AS} $(MP_EXTRA_AS_PRE)  "../DEE Emulation 16-bit/Flash Operations.s" -o "${OBJECTDIR}/_ext/869223802/Flash Operations.o" -omf=elf -p=$(MP_PROCESSOR_OPTION) --defsym=__MPLAB_BUILD=1 --defsym=__MPLAB_DEBUG=1 --defsym=__ICD2RAM=1 --defsym=__DEBUG=1 --defsym=__MPLAB_DEBUGGER_ICD3=1 -g  -MD "${OBJECTDIR}/_ext/869223802/Flash Operations.o.d" -I".." -g $(MP_EXTRA_AS_POST)
	
else
${OBJECTDIR}/_ext/869223802/Flash\ Operations.o: ../DEE\ Emulation\ 16-bit/Flash\ Operations.s  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/869223802" 
	@${RM} "${OBJECTDIR}/_ext/869223802/Flash Operations.o".d 
	@${RM} "${OBJECTDIR}/_ext/869223802/Flash Operations.o".ok ${OBJECTDIR}/_ext/869223802/Flash\ Operations.o.err 
	@${RM} "${OBJECTDIR}/_ext/869223802/Flash Operations.o" 
	@${FIXDEPS} "${OBJECTDIR}/_ext/869223802/Flash Operations.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_AS} $(MP_EXTRA_AS_PRE)  "../DEE Emulation 16-bit/Flash Operations.s" -o "${OBJECTDIR}/_ext/869223802/Flash Operations.o" -omf=elf -p=$(MP_PROCESSOR_OPTION) --defsym=__MPLAB_BUILD=1 -g  -MD "${OBJECTDIR}/_ext/869223802/Flash Operations.o.d" -I".." -g $(MP_EXTRA_AS_POST)
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assembleWithPreprocess
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: compile
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/_ext/2108356922/uart2.o: ../Common/uart2.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/2108356922" 
	@${RM} ${OBJECTDIR}/_ext/2108356922/uart2.o.d 
	@${RM} ${OBJECTDIR}/_ext/2108356922/uart2.o.ok ${OBJECTDIR}/_ext/2108356922/uart2.o.err 
	@${RM} ${OBJECTDIR}/_ext/2108356922/uart2.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/2108356922/uart2.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -I".." -I"../USB" -I"../Common" -mconst-in-code -Os -fschedule-insns -fschedule-insns2 -MMD -MF "${OBJECTDIR}/_ext/2108356922/uart2.o.d" -o ${OBJECTDIR}/_ext/2108356922/uart2.o ../Common/uart2.c    
	
${OBJECTDIR}/_ext/869223802/DEE\ Emulation\ 16-bit.o: ../DEE\ Emulation\ 16-bit/DEE\ Emulation\ 16-bit.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/869223802" 
	@${RM} "${OBJECTDIR}/_ext/869223802/DEE Emulation 16-bit.o".d 
	@${RM} "${OBJECTDIR}/_ext/869223802/DEE Emulation 16-bit.o".ok ${OBJECTDIR}/_ext/869223802/DEE\ Emulation\ 16-bit.o.err 
	@${RM} "${OBJECTDIR}/_ext/869223802/DEE Emulation 16-bit.o" 
	@${FIXDEPS} "${OBJECTDIR}/_ext/869223802/DEE Emulation 16-bit.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -I".." -I"../USB" -I"../Common" -mconst-in-code -Os -fschedule-insns -fschedule-insns2 -MMD -MF "${OBJECTDIR}/_ext/869223802/DEE Emulation 16-bit.o.d" -o "${OBJECTDIR}/_ext/869223802/DEE Emulation 16-bit.o" "../DEE Emulation 16-bit/DEE Emulation 16-bit.c"    
	
${OBJECTDIR}/_ext/1360907413/usb_host.o: ../USB/usb_host.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360907413" 
	@${RM} ${OBJECTDIR}/_ext/1360907413/usb_host.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360907413/usb_host.o.ok ${OBJECTDIR}/_ext/1360907413/usb_host.o.err 
	@${RM} ${OBJECTDIR}/_ext/1360907413/usb_host.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360907413/usb_host.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -I".." -I"../USB" -I"../Common" -mconst-in-code -Os -fschedule-insns -fschedule-insns2 -MMD -MF "${OBJECTDIR}/_ext/1360907413/usb_host.o.d" -o ${OBJECTDIR}/_ext/1360907413/usb_host.o ../USB/usb_host.c    
	
${OBJECTDIR}/_ext/1360907413/usb_host_generic_PS4.o: ../USB/usb_host_generic_PS4.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360907413" 
	@${RM} ${OBJECTDIR}/_ext/1360907413/usb_host_generic_PS4.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360907413/usb_host_generic_PS4.o.ok ${OBJECTDIR}/_ext/1360907413/usb_host_generic_PS4.o.err 
	@${RM} ${OBJECTDIR}/_ext/1360907413/usb_host_generic_PS4.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360907413/usb_host_generic_PS4.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -I".." -I"../USB" -I"../Common" -mconst-in-code -Os -fschedule-insns -fschedule-insns2 -MMD -MF "${OBJECTDIR}/_ext/1360907413/usb_host_generic_PS4.o.d" -o ${OBJECTDIR}/_ext/1360907413/usb_host_generic_PS4.o ../USB/usb_host_generic_PS4.c    
	
${OBJECTDIR}/_ext/1472/main.o: ../main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/main.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/main.o.ok ${OBJECTDIR}/_ext/1472/main.o.err 
	@${RM} ${OBJECTDIR}/_ext/1472/main.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/main.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -I".." -I"../USB" -I"../Common" -mconst-in-code -Os -fschedule-insns -fschedule-insns2 -MMD -MF "${OBJECTDIR}/_ext/1472/main.o.d" -o ${OBJECTDIR}/_ext/1472/main.o ../main.c    
	
${OBJECTDIR}/_ext/1472/usb_config.o: ../usb_config.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/usb_config.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/usb_config.o.ok ${OBJECTDIR}/_ext/1472/usb_config.o.err 
	@${RM} ${OBJECTDIR}/_ext/1472/usb_config.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/usb_config.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -I".." -I"../USB" -I"../Common" -mconst-in-code -Os -fschedule-insns -fschedule-insns2 -MMD -MF "${OBJECTDIR}/_ext/1472/usb_config.o.d" -o ${OBJECTDIR}/_ext/1472/usb_config.o ../usb_config.c    
	
${OBJECTDIR}/_ext/1472/outputs.o: ../outputs.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/outputs.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/outputs.o.ok ${OBJECTDIR}/_ext/1472/outputs.o.err 
	@${RM} ${OBJECTDIR}/_ext/1472/outputs.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/outputs.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -I".." -I"../USB" -I"../Common" -mconst-in-code -Os -fschedule-insns -fschedule-insns2 -MMD -MF "${OBJECTDIR}/_ext/1472/outputs.o.d" -o ${OBJECTDIR}/_ext/1472/outputs.o ../outputs.c    
	
${OBJECTDIR}/_ext/1472/functional_test.o: ../functional_test.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/functional_test.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/functional_test.o.ok ${OBJECTDIR}/_ext/1472/functional_test.o.err 
	@${RM} ${OBJECTDIR}/_ext/1472/functional_test.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/functional_test.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -I".." -I"../USB" -I"../Common" -mconst-in-code -Os -fschedule-insns -fschedule-insns2 -MMD -MF "${OBJECTDIR}/_ext/1472/functional_test.o.d" -o ${OBJECTDIR}/_ext/1472/functional_test.o ../functional_test.c    
	
${OBJECTDIR}/_ext/1472/PS4_SPI.o: ../PS4_SPI.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/PS4_SPI.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/PS4_SPI.o.ok ${OBJECTDIR}/_ext/1472/PS4_SPI.o.err 
	@${RM} ${OBJECTDIR}/_ext/1472/PS4_SPI.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/PS4_SPI.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -I".." -I"../USB" -I"../Common" -mconst-in-code -Os -fschedule-insns -fschedule-insns2 -MMD -MF "${OBJECTDIR}/_ext/1472/PS4_SPI.o.d" -o ${OBJECTDIR}/_ext/1472/PS4_SPI.o ../PS4_SPI.c    
	
${OBJECTDIR}/_ext/1472/PS4_controller.o: ../PS4_controller.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/PS4_controller.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/PS4_controller.o.ok ${OBJECTDIR}/_ext/1472/PS4_controller.o.err 
	@${RM} ${OBJECTDIR}/_ext/1472/PS4_controller.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/PS4_controller.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -I".." -I"../USB" -I"../Common" -mconst-in-code -Os -fschedule-insns -fschedule-insns2 -MMD -MF "${OBJECTDIR}/_ext/1472/PS4_controller.o.d" -o ${OBJECTDIR}/_ext/1472/PS4_controller.o ../PS4_controller.c    
	
else
${OBJECTDIR}/_ext/2108356922/uart2.o: ../Common/uart2.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/2108356922" 
	@${RM} ${OBJECTDIR}/_ext/2108356922/uart2.o.d 
	@${RM} ${OBJECTDIR}/_ext/2108356922/uart2.o.ok ${OBJECTDIR}/_ext/2108356922/uart2.o.err 
	@${RM} ${OBJECTDIR}/_ext/2108356922/uart2.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/2108356922/uart2.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -I".." -I"../USB" -I"../Common" -mconst-in-code -Os -fschedule-insns -fschedule-insns2 -MMD -MF "${OBJECTDIR}/_ext/2108356922/uart2.o.d" -o ${OBJECTDIR}/_ext/2108356922/uart2.o ../Common/uart2.c    
	
${OBJECTDIR}/_ext/869223802/DEE\ Emulation\ 16-bit.o: ../DEE\ Emulation\ 16-bit/DEE\ Emulation\ 16-bit.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/869223802" 
	@${RM} "${OBJECTDIR}/_ext/869223802/DEE Emulation 16-bit.o".d 
	@${RM} "${OBJECTDIR}/_ext/869223802/DEE Emulation 16-bit.o".ok ${OBJECTDIR}/_ext/869223802/DEE\ Emulation\ 16-bit.o.err 
	@${RM} "${OBJECTDIR}/_ext/869223802/DEE Emulation 16-bit.o" 
	@${FIXDEPS} "${OBJECTDIR}/_ext/869223802/DEE Emulation 16-bit.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -I".." -I"../USB" -I"../Common" -mconst-in-code -Os -fschedule-insns -fschedule-insns2 -MMD -MF "${OBJECTDIR}/_ext/869223802/DEE Emulation 16-bit.o.d" -o "${OBJECTDIR}/_ext/869223802/DEE Emulation 16-bit.o" "../DEE Emulation 16-bit/DEE Emulation 16-bit.c"    
	
${OBJECTDIR}/_ext/1360907413/usb_host.o: ../USB/usb_host.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360907413" 
	@${RM} ${OBJECTDIR}/_ext/1360907413/usb_host.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360907413/usb_host.o.ok ${OBJECTDIR}/_ext/1360907413/usb_host.o.err 
	@${RM} ${OBJECTDIR}/_ext/1360907413/usb_host.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360907413/usb_host.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -I".." -I"../USB" -I"../Common" -mconst-in-code -Os -fschedule-insns -fschedule-insns2 -MMD -MF "${OBJECTDIR}/_ext/1360907413/usb_host.o.d" -o ${OBJECTDIR}/_ext/1360907413/usb_host.o ../USB/usb_host.c    
	
${OBJECTDIR}/_ext/1360907413/usb_host_generic_PS4.o: ../USB/usb_host_generic_PS4.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360907413" 
	@${RM} ${OBJECTDIR}/_ext/1360907413/usb_host_generic_PS4.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360907413/usb_host_generic_PS4.o.ok ${OBJECTDIR}/_ext/1360907413/usb_host_generic_PS4.o.err 
	@${RM} ${OBJECTDIR}/_ext/1360907413/usb_host_generic_PS4.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360907413/usb_host_generic_PS4.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -I".." -I"../USB" -I"../Common" -mconst-in-code -Os -fschedule-insns -fschedule-insns2 -MMD -MF "${OBJECTDIR}/_ext/1360907413/usb_host_generic_PS4.o.d" -o ${OBJECTDIR}/_ext/1360907413/usb_host_generic_PS4.o ../USB/usb_host_generic_PS4.c    
	
${OBJECTDIR}/_ext/1472/main.o: ../main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/main.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/main.o.ok ${OBJECTDIR}/_ext/1472/main.o.err 
	@${RM} ${OBJECTDIR}/_ext/1472/main.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/main.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -I".." -I"../USB" -I"../Common" -mconst-in-code -Os -fschedule-insns -fschedule-insns2 -MMD -MF "${OBJECTDIR}/_ext/1472/main.o.d" -o ${OBJECTDIR}/_ext/1472/main.o ../main.c    
	
${OBJECTDIR}/_ext/1472/usb_config.o: ../usb_config.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/usb_config.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/usb_config.o.ok ${OBJECTDIR}/_ext/1472/usb_config.o.err 
	@${RM} ${OBJECTDIR}/_ext/1472/usb_config.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/usb_config.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -I".." -I"../USB" -I"../Common" -mconst-in-code -Os -fschedule-insns -fschedule-insns2 -MMD -MF "${OBJECTDIR}/_ext/1472/usb_config.o.d" -o ${OBJECTDIR}/_ext/1472/usb_config.o ../usb_config.c    
	
${OBJECTDIR}/_ext/1472/outputs.o: ../outputs.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/outputs.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/outputs.o.ok ${OBJECTDIR}/_ext/1472/outputs.o.err 
	@${RM} ${OBJECTDIR}/_ext/1472/outputs.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/outputs.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -I".." -I"../USB" -I"../Common" -mconst-in-code -Os -fschedule-insns -fschedule-insns2 -MMD -MF "${OBJECTDIR}/_ext/1472/outputs.o.d" -o ${OBJECTDIR}/_ext/1472/outputs.o ../outputs.c    
	
${OBJECTDIR}/_ext/1472/functional_test.o: ../functional_test.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/functional_test.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/functional_test.o.ok ${OBJECTDIR}/_ext/1472/functional_test.o.err 
	@${RM} ${OBJECTDIR}/_ext/1472/functional_test.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/functional_test.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -I".." -I"../USB" -I"../Common" -mconst-in-code -Os -fschedule-insns -fschedule-insns2 -MMD -MF "${OBJECTDIR}/_ext/1472/functional_test.o.d" -o ${OBJECTDIR}/_ext/1472/functional_test.o ../functional_test.c    
	
${OBJECTDIR}/_ext/1472/PS4_SPI.o: ../PS4_SPI.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/PS4_SPI.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/PS4_SPI.o.ok ${OBJECTDIR}/_ext/1472/PS4_SPI.o.err 
	@${RM} ${OBJECTDIR}/_ext/1472/PS4_SPI.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/PS4_SPI.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -I".." -I"../USB" -I"../Common" -mconst-in-code -Os -fschedule-insns -fschedule-insns2 -MMD -MF "${OBJECTDIR}/_ext/1472/PS4_SPI.o.d" -o ${OBJECTDIR}/_ext/1472/PS4_SPI.o ../PS4_SPI.c    
	
${OBJECTDIR}/_ext/1472/PS4_controller.o: ../PS4_controller.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/PS4_controller.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/PS4_controller.o.ok ${OBJECTDIR}/_ext/1472/PS4_controller.o.err 
	@${RM} ${OBJECTDIR}/_ext/1472/PS4_controller.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/PS4_controller.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -I".." -I"../USB" -I"../Common" -mconst-in-code -Os -fschedule-insns -fschedule-insns2 -MMD -MF "${OBJECTDIR}/_ext/1472/PS4_controller.o.d" -o ${OBJECTDIR}/_ext/1472/PS4_controller.o ../PS4_controller.c    
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: link
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
dist/${CND_CONF}/${IMAGE_TYPE}/PIC_PS4_Host.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk    
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -omf=elf -mcpu=$(MP_PROCESSOR_OPTION)  -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -o dist/${CND_CONF}/${IMAGE_TYPE}/PIC_PS4_Host.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}         -Wl,--defsym=__MPLAB_BUILD=1,--heap=10000,-Map="${DISTDIR}/general.X.${IMAGE_TYPE}.map",--report-mem$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION),--defsym=__ICD2RAM=1,--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,--defsym=__MPLAB_DEBUGGER_ICD3=1
else
dist/${CND_CONF}/${IMAGE_TYPE}/PIC_PS4_Host.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk   
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -omf=elf -mcpu=$(MP_PROCESSOR_OPTION)  -o dist/${CND_CONF}/${IMAGE_TYPE}/PIC_PS4_Host.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}         -Wl,--defsym=__MPLAB_BUILD=1,--heap=10000,-Map="${DISTDIR}/general.X.${IMAGE_TYPE}.map",--report-mem$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION)
	${MP_CC_DIR}\\pic30-bin2hex dist/${CND_CONF}/${IMAGE_TYPE}/PIC_PS4_Host.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} -omf=elf
endif


# Subprojects
.build-subprojects:


# Subprojects
.clean-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r build/default
	${RM} -r dist/default

# Enable dependency checking
.dep.inc: .depcheck-impl

DEPFILES=$(shell mplabwildcard ${POSSIBLE_DEPFILES})
ifneq (${DEPFILES},)
include ${DEPFILES}
endif
