################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
"../code/app/balance_app.c" \
"../code/app/balance_slow_app.c" \
"../code/app/imu_app.c" \
"../code/app/motor_app.c" \
"../code/app/nav_app.c" \
"../code/app/schedule.c" \
"../code/app/servo_app.c" \
"../code/app/uart_command.c" 

COMPILED_SRCS += \
"code/app/balance_app.src" \
"code/app/balance_slow_app.src" \
"code/app/imu_app.src" \
"code/app/motor_app.src" \
"code/app/nav_app.src" \
"code/app/schedule.src" \
"code/app/servo_app.src" \
"code/app/uart_command.src" 

C_DEPS += \
"./code/app/balance_app.d" \
"./code/app/balance_slow_app.d" \
"./code/app/imu_app.d" \
"./code/app/motor_app.d" \
"./code/app/nav_app.d" \
"./code/app/schedule.d" \
"./code/app/servo_app.d" \
"./code/app/uart_command.d" 

OBJS += \
"code/app/balance_app.o" \
"code/app/balance_slow_app.o" \
"code/app/imu_app.o" \
"code/app/motor_app.o" \
"code/app/nav_app.o" \
"code/app/schedule.o" \
"code/app/servo_app.o" \
"code/app/uart_command.o" 


# Each subdirectory must supply rules for building sources it contributes
"code/app/balance_app.src":"../code/app/balance_app.c" "code/app/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2012 -D__CPU__=tc26xb "-fC:/Users/wjj/Desktop/111/Bic/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
"code/app/balance_app.o":"code/app/balance_app.src" "code/app/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"code/app/balance_slow_app.src":"../code/app/balance_slow_app.c" "code/app/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2012 -D__CPU__=tc26xb "-fC:/Users/wjj/Desktop/111/Bic/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
"code/app/balance_slow_app.o":"code/app/balance_slow_app.src" "code/app/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"code/app/imu_app.src":"../code/app/imu_app.c" "code/app/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2012 -D__CPU__=tc26xb "-fC:/Users/wjj/Desktop/111/Bic/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
"code/app/imu_app.o":"code/app/imu_app.src" "code/app/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"code/app/motor_app.src":"../code/app/motor_app.c" "code/app/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2012 -D__CPU__=tc26xb "-fC:/Users/wjj/Desktop/111/Bic/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
"code/app/motor_app.o":"code/app/motor_app.src" "code/app/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"code/app/nav_app.src":"../code/app/nav_app.c" "code/app/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2012 -D__CPU__=tc26xb "-fC:/Users/wjj/Desktop/111/Bic/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
"code/app/nav_app.o":"code/app/nav_app.src" "code/app/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"code/app/schedule.src":"../code/app/schedule.c" "code/app/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2012 -D__CPU__=tc26xb "-fC:/Users/wjj/Desktop/111/Bic/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
"code/app/schedule.o":"code/app/schedule.src" "code/app/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"code/app/servo_app.src":"../code/app/servo_app.c" "code/app/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2012 -D__CPU__=tc26xb "-fC:/Users/wjj/Desktop/111/Bic/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
"code/app/servo_app.o":"code/app/servo_app.src" "code/app/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"code/app/uart_command.src":"../code/app/uart_command.c" "code/app/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2012 -D__CPU__=tc26xb "-fC:/Users/wjj/Desktop/111/Bic/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
"code/app/uart_command.o":"code/app/uart_command.src" "code/app/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"

clean: clean-code-2f-app

clean-code-2f-app:
	-$(RM) ./code/app/balance_app.d ./code/app/balance_app.o ./code/app/balance_app.src ./code/app/balance_slow_app.d ./code/app/balance_slow_app.o ./code/app/balance_slow_app.src ./code/app/imu_app.d ./code/app/imu_app.o ./code/app/imu_app.src ./code/app/motor_app.d ./code/app/motor_app.o ./code/app/motor_app.src ./code/app/nav_app.d ./code/app/nav_app.o ./code/app/nav_app.src ./code/app/schedule.d ./code/app/schedule.o ./code/app/schedule.src ./code/app/servo_app.d ./code/app/servo_app.o ./code/app/servo_app.src ./code/app/uart_command.d ./code/app/uart_command.o ./code/app/uart_command.src

.PHONY: clean-code-2f-app

