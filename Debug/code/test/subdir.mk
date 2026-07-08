################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
"../code/test/debug_telemetry.c" \
"../code/test/nav_manual_test.c" \
"../code/test/nav_route_test.c" 

COMPILED_SRCS += \
"code/test/debug_telemetry.src" \
"code/test/nav_manual_test.src" \
"code/test/nav_route_test.src" 

C_DEPS += \
"./code/test/debug_telemetry.d" \
"./code/test/nav_manual_test.d" \
"./code/test/nav_route_test.d" 

OBJS += \
"code/test/debug_telemetry.o" \
"code/test/nav_manual_test.o" \
"code/test/nav_route_test.o" 


# Each subdirectory must supply rules for building sources it contributes
"code/test/debug_telemetry.src":"../code/test/debug_telemetry.c" "code/test/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2012 -D__CPU__=tc26xb "-fC:/Users/wjj/Desktop/111/Bic/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
"code/test/debug_telemetry.o":"code/test/debug_telemetry.src" "code/test/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"code/test/nav_manual_test.src":"../code/test/nav_manual_test.c" "code/test/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2012 -D__CPU__=tc26xb "-fC:/Users/wjj/Desktop/111/Bic/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
"code/test/nav_manual_test.o":"code/test/nav_manual_test.src" "code/test/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"code/test/nav_route_test.src":"../code/test/nav_route_test.c" "code/test/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2012 -D__CPU__=tc26xb "-fC:/Users/wjj/Desktop/111/Bic/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
"code/test/nav_route_test.o":"code/test/nav_route_test.src" "code/test/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"

clean: clean-code-2f-test

clean-code-2f-test:
	-$(RM) ./code/test/debug_telemetry.d ./code/test/debug_telemetry.o ./code/test/debug_telemetry.src ./code/test/nav_manual_test.d ./code/test/nav_manual_test.o ./code/test/nav_manual_test.src ./code/test/nav_route_test.d ./code/test/nav_route_test.o ./code/test/nav_route_test.src

.PHONY: clean-code-2f-test

