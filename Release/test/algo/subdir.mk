################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../test/algo/dijkstra.cpp \
../test/algo/eppstein.cpp \
../test/algo/sp_range_query.cpp \
../test/algo/topological_sort.cpp 

OBJS += \
./test/algo/dijkstra.o \
./test/algo/eppstein.o \
./test/algo/sp_range_query.o \
./test/algo/topological_sort.o 

CPP_DEPS += \
./test/algo/dijkstra.d \
./test/algo/eppstein.d \
./test/algo/sp_range_query.d \
./test/algo/topological_sort.d 


# Each subdirectory must supply rules for building sources it contributes
test/algo/%.o: ../test/algo/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -O3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


