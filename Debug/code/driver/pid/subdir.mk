################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
"../code/driver/pid/pid_driver.c" 

COMPILED_SRCS += \
"code/driver/pid/pid_driver.src" 

C_DEPS += \
"./code/driver/pid/pid_driver.d" 

OBJS += \
"code/driver/pid/pid_driver.o" 


# Each subdirectory must supply rules for building sources it contributes
"code/driver/pid/pid_driver.src":"../code/driver/pid/pid_driver.c" "code/driver/pid/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2012 -D__CPU__=tc26xb "-fC:/Users/wjj/Desktop/111/Bic/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
"code/driver/pid/pid_driver.o":"code/driver/pid/pid_driver.src" "code/driver/pid/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"

clean: clean-code-2f-driver-2f-pid

clean-code-2f-driver-2f-pid:
	-$(RM) ./code/driver/pid/pid_driver.d ./code/driver/pid/pid_driver.o ./code/driver/pid/pid_driver.src

.PHONY: clean-code-2f-driver-2f-pid

