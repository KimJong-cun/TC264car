################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
"../libraries/zf_device/st/lsm6dsv16x_reg.c" 

COMPILED_SRCS += \
"libraries/zf_device/st/lsm6dsv16x_reg.src" 

C_DEPS += \
"./libraries/zf_device/st/lsm6dsv16x_reg.d" 

OBJS += \
"libraries/zf_device/st/lsm6dsv16x_reg.o" 


# Each subdirectory must supply rules for building sources it contributes
"libraries/zf_device/st/lsm6dsv16x_reg.src":"../libraries/zf_device/st/lsm6dsv16x_reg.c" "libraries/zf_device/st/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2012 -D__CPU__=tc26xb "-fC:/Users/suiyungui/Desktop/Seekfree_TC264_Opensource_Library/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
"libraries/zf_device/st/lsm6dsv16x_reg.o":"libraries/zf_device/st/lsm6dsv16x_reg.src" "libraries/zf_device/st/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"

clean: clean-libraries-2f-zf_device-2f-st

clean-libraries-2f-zf_device-2f-st:
	-$(RM) ./libraries/zf_device/st/lsm6dsv16x_reg.d ./libraries/zf_device/st/lsm6dsv16x_reg.o ./libraries/zf_device/st/lsm6dsv16x_reg.src

.PHONY: clean-libraries-2f-zf_device-2f-st

