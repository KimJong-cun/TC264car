################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
"../code/driver/imu/ekf.c" \
"../code/driver/imu/matrix.c" 

COMPILED_SRCS += \
"code/driver/imu/ekf.src" \
"code/driver/imu/matrix.src" 

C_DEPS += \
"./code/driver/imu/ekf.d" \
"./code/driver/imu/matrix.d" 

OBJS += \
"code/driver/imu/ekf.o" \
"code/driver/imu/matrix.o" 


# Each subdirectory must supply rules for building sources it contributes
"code/driver/imu/ekf.src":"../code/driver/imu/ekf.c" "code/driver/imu/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2012 -D__CPU__=tc26xb "-fC:/Users/wjj/Desktop/111/Bic/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
"code/driver/imu/ekf.o":"code/driver/imu/ekf.src" "code/driver/imu/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"code/driver/imu/matrix.src":"../code/driver/imu/matrix.c" "code/driver/imu/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2012 -D__CPU__=tc26xb "-fC:/Users/wjj/Desktop/111/Bic/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
"code/driver/imu/matrix.o":"code/driver/imu/matrix.src" "code/driver/imu/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"

clean: clean-code-2f-driver-2f-imu

clean-code-2f-driver-2f-imu:
	-$(RM) ./code/driver/imu/ekf.d ./code/driver/imu/ekf.o ./code/driver/imu/ekf.src ./code/driver/imu/matrix.d ./code/driver/imu/matrix.o ./code/driver/imu/matrix.src

.PHONY: clean-code-2f-driver-2f-imu

