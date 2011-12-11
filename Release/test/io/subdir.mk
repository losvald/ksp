################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../test/io/graph_io.cpp 

OBJS += \
./test/io/graph_io.o 

CPP_DEPS += \
./test/io/graph_io.d 


# Each subdirectory must supply rules for building sources it contributes
test/io/%.o: ../test/io/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -URUN_TESTS_ENABLED -O3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


