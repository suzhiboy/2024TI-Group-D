################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Add inputs and outputs from these tool invocations to the build variables 
SYSCFG_SRCS += \
../gpio_toggle_output.syscfg 

C_SRCS += \
../control.c \
../debug.c \
../delay.c \
../gpio_toggle_output.c \
./ti_msp_dl_config.c \
C:/TI/mspm0_sdk_2_10_00_04/source/ti/devices/msp/m0p/startup_system_files/ticlang/startup_mspm0g350x_ticlang.c \
../motor.c \
../mpu6050.c \
../pid.c \
../sensor.c 

GEN_CMDS += \
./device_linker.cmd 

GEN_FILES += \
./device_linker.cmd \
./device.opt \
./ti_msp_dl_config.c 

C_DEPS += \
./control.d \
./debug.d \
./delay.d \
./gpio_toggle_output.d \
./ti_msp_dl_config.d \
./startup_mspm0g350x_ticlang.d \
./motor.d \
./mpu6050.d \
./pid.d \
./sensor.d 

GEN_OPTS += \
./device.opt 

OBJS += \
./control.o \
./debug.o \
./delay.o \
./gpio_toggle_output.o \
./ti_msp_dl_config.o \
./startup_mspm0g350x_ticlang.o \
./motor.o \
./mpu6050.o \
./pid.o \
./sensor.o 

GEN_MISC_FILES += \
./device.cmd.genlibs \
./ti_msp_dl_config.h \
./Event.dot 

OBJS__QUOTED += \
"control.o" \
"debug.o" \
"delay.o" \
"gpio_toggle_output.o" \
"ti_msp_dl_config.o" \
"startup_mspm0g350x_ticlang.o" \
"motor.o" \
"mpu6050.o" \
"pid.o" \
"sensor.o" 

GEN_MISC_FILES__QUOTED += \
"device.cmd.genlibs" \
"ti_msp_dl_config.h" \
"Event.dot" 

C_DEPS__QUOTED += \
"control.d" \
"debug.d" \
"delay.d" \
"gpio_toggle_output.d" \
"ti_msp_dl_config.d" \
"startup_mspm0g350x_ticlang.d" \
"motor.d" \
"mpu6050.d" \
"pid.d" \
"sensor.d" 

GEN_FILES__QUOTED += \
"device_linker.cmd" \
"device.opt" \
"ti_msp_dl_config.c" 

C_SRCS__QUOTED += \
"../control.c" \
"../debug.c" \
"../delay.c" \
"../gpio_toggle_output.c" \
"./ti_msp_dl_config.c" \
"C:/TI/mspm0_sdk_2_10_00_04/source/ti/devices/msp/m0p/startup_system_files/ticlang/startup_mspm0g350x_ticlang.c" \
"../motor.c" \
"../mpu6050.c" \
"../pid.c" \
"../sensor.c" 

SYSCFG_SRCS__QUOTED += \
"../gpio_toggle_output.syscfg" 


