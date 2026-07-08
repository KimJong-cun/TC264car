################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
"../code/driver/motor/motor_uart_driver.c" 

COMPILED_SRCS += \
"code/driver/motor/motor_uart_driver.src" 

C_DEPS += \
"./code/driver/motor/motor_uart_driver.d" 

OBJS += \
"code/driver/motor/motor_uart_driver.o" 


# Each subdirectory must supply rules for building sources it contributes
"code/driver/motor/motor_uart_driver.src":"../code/driver/motor/motor_uart_driver.c" "code/driver/motor/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2012 -D__CPU__=tc26xb "-fC:/Users/wjj/Desktop/111/Bic/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
"code/driver/motor/motor_uart_driver.o":"code/driver/motor/motor_uart_driver.src" "code/driver/motor/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"

clean: clean-code-2f-driver-2f-motor

clean-code-2f-driver-2f-motor:
	-$(RM) ./code/driver/motor/motor_uart_driver.d ./code/driver/motor/motor_uart_driver.o ./code/driver/motor/motor_uart_driver.src

.PHONY: clean-code-2f-driver-2f-motor

