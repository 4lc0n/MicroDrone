################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Source/adc.c \
../Source/i2cmaster.c \
../Source/light_ws2812.c \
../Source/main.c \
../Source/uart.c 

OBJS += \
./Source/adc.o \
./Source/i2cmaster.o \
./Source/light_ws2812.o \
./Source/main.o \
./Source/uart.o 

C_DEPS += \
./Source/adc.d \
./Source/i2cmaster.d \
./Source/light_ws2812.d \
./Source/main.d \
./Source/uart.d 


# Each subdirectory must supply rules for building sources it contributes
Source/%.o: ../Source/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: AVR Compiler'
	avr-gcc -Wall -O1 -fpack-struct -fshort-enums -ffunction-sections -fdata-sections -std=gnu99 -funsigned-char -funsigned-bitfields -mmcu=atmega328p -DF_CPU=16000000UL -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


