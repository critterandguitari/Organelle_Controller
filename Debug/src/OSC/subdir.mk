################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/OSC/OSCData.cpp \
../src/OSC/OSCMessage.cpp \
../src/OSC/OSCTiming.cpp \
../src/OSC/SimpleWriter.cpp 

C_SRCS += \
../src/OSC/OSCMatch.c 

OBJS += \
./src/OSC/OSCData.o \
./src/OSC/OSCMatch.o \
./src/OSC/OSCMessage.o \
./src/OSC/OSCTiming.o \
./src/OSC/SimpleWriter.o 

C_DEPS += \
./src/OSC/OSCMatch.d 

CPP_DEPS += \
./src/OSC/OSCData.d \
./src/OSC/OSCMessage.d \
./src/OSC/OSCTiming.d \
./src/OSC/SimpleWriter.d 


# Each subdirectory must supply rules for building sources it contributes
src/OSC/%.o: ../src/OSC/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C++ Compiler'
	arm-none-eabi-g++ -mcpu=cortex-m0 -mthumb -Og -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -Wall -Wextra  -g3 -DDEBUG -DUSE_FULL_ASSERT -DSTM32F051 -DUSE_STDPERIPH_DRIVER -DHSE_VALUE=8000000 -I"../include" -I"../system/include" -I"../system/include/cmsis" -I"../system/include/stm32f0-stdperiph" -std=gnu++11 -fabi-version=0 -fno-exceptions -fno-rtti -fno-use-cxa-atexit -fno-threadsafe-statics -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

src/OSC/%.o: ../src/OSC/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -Og -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -Wall -Wextra  -g3 -DDEBUG -DUSE_FULL_ASSERT -DSTM32F051 -DUSE_STDPERIPH_DRIVER -DHSE_VALUE=8000000 -I"../include" -I"../system/include" -I"../system/include/cmsis" -I"../system/include/stm32f0-stdperiph" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


