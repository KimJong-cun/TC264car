################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
"../code/driver/uart/uart_cmd_line.c" \
"../code/driver/uart/uart_cmd_rx.c" 

COMPILED_SRCS += \
"code/driver/uart/uart_cmd_line.src" \
"code/driver/uart/uart_cmd_rx.src" 

C_DEPS += \
"./code/driver/uart/uart_cmd_line.d" \
"./code/driver/uart/uart_cmd_rx.d" 

OBJS += \
"code/driver/uart/uart_cmd_line.o" \
"code/driver/uart/uart_cmd_rx.o" 


# Each subdirectory must supply rules for building sources it contributes
"code/driver/uart/uart_cmd_line.src":"../code/driver/uart/uart_cmd_line.c" "code/driver/uart/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2012 -D__CPU__=tc26xb "-fC:/Users/wjj/Desktop/111/Bic/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
"code/driver/uart/uart_cmd_line.o":"code/driver/uart/uart_cmd_line.src" "code/driver/uart/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"code/driver/uart/uart_cmd_rx.src":"../code/driver/uart/uart_cmd_rx.c" "code/driver/uart/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2012 -D__CPU__=tc26xb "-fC:/Users/wjj/Desktop/111/Bic/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
"code/driver/uart/uart_cmd_rx.o":"code/driver/uart/uart_cmd_rx.src" "code/driver/uart/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"

clean: clean-code-2f-driver-2f-uart

clean-code-2f-driver-2f-uart:
	-$(RM) ./code/driver/uart/uart_cmd_line.d ./code/driver/uart/uart_cmd_line.o ./code/driver/uart/uart_cmd_line.src ./code/driver/uart/uart_cmd_rx.d ./code/driver/uart/uart_cmd_rx.o ./code/driver/uart/uart_cmd_rx.src

.PHONY: clean-code-2f-driver-2f-uart

