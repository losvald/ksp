################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../test/adjacency_list.cpp \
../test/gen.cpp \
../test/graph_builder.cpp \
../test/interactive_frontend.cpp \
../test/shortest_path_tree.cpp \
../test/test.cpp 

OBJS += \
./test/adjacency_list.o \
./test/gen.o \
./test/graph_builder.o \
./test/interactive_frontend.o \
./test/shortest_path_tree.o \
./test/test.o 

CPP_DEPS += \
./test/adjacency_list.d \
./test/gen.d \
./test/graph_builder.d \
./test/interactive_frontend.d \
./test/shortest_path_tree.d \
./test/test.d 


# Each subdirectory must supply rules for building sources it contributes
test/%.o: ../test/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -URUN_TESTS_ENABLED -O3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


