################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../test/gen/random_utils.cpp 

OBJS += \
./test/gen/random_utils.o 

CPP_DEPS += \
./test/gen/random_utils.d 


# Each subdirectory must supply rules for building sources it contributes
test/gen/%.o: ../test/gen/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -DDEBUG -URUN_TESTS_ENABLED -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

