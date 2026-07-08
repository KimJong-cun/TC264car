################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
"../code/driver/nav/nav_manual.c" \
"../code/driver/nav/nav_odometry.c" \
"../code/driver/nav/nav_route.c" 

COMPILED_SRCS += \
"code/driver/nav/nav_manual.src" \
"code/driver/nav/nav_odometry.src" \
"code/driver/nav/nav_route.src" 

C_DEPS += \
"./code/driver/nav/nav_manual.d" \
"./code/driver/nav/nav_odometry.d" \
"./code/driver/nav/nav_route.d" 

OBJS += \
"code/driver/nav/nav_manual.o" \
"code/driver/nav/nav_odometry.o" \
"code/driver/nav/nav_route.o" 


# Each subdirectory must supply rules for building sources it contributes
"code/driver/nav/nav_manual.src":"../code/driver/nav/nav_manual.c" "code/driver/nav/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2012 -D__CPU__=tc26xb "-fC:/Users/wjj/Desktop/111/Bic/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
"code/driver/nav/nav_manual.o":"code/driver/nav/nav_manual.src" "code/driver/nav/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"code/driver/nav/nav_odometry.src":"../code/driver/nav/nav_odometry.c" "code/driver/nav/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2012 -D__CPU__=tc26xb "-fC:/Users/wjj/Desktop/111/Bic/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
"code/driver/nav/nav_odometry.o":"code/driver/nav/nav_odometry.src" "code/driver/nav/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"code/driver/nav/nav_route.src":"../code/driver/nav/nav_route.c" "code/driver/nav/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2012 -D__CPU__=tc26xb "-fC:/Users/wjj/Desktop/111/Bic/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
"code/driver/nav/nav_route.o":"code/driver/nav/nav_route.src" "code/driver/nav/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"

clean: clean-code-2f-driver-2f-nav

clean-code-2f-driver-2f-nav:
	-$(RM) ./code/driver/nav/nav_manual.d ./code/driver/nav/nav_manual.o ./code/driver/nav/nav_manual.src ./code/driver/nav/nav_odometry.d ./code/driver/nav/nav_odometry.o ./code/driver/nav/nav_odometry.src ./code/driver/nav/nav_route.d ./code/driver/nav/nav_route.o ./code/driver/nav/nav_route.src

.PHONY: clean-code-2f-driver-2f-nav

