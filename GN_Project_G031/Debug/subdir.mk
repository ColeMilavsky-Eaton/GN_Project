################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../main.c \
../stm32g0xx_hal_msp.c \
../stm32g0xx_it.c \
../syscalls.c \
../sysmem.c \
../system_stm32g0xx.c 

OBJS += \
./main.o \
./stm32g0xx_hal_msp.o \
./stm32g0xx_it.o \
./syscalls.o \
./sysmem.o \
./system_stm32g0xx.o 

C_DEPS += \
./main.d \
./stm32g0xx_hal_msp.d \
./stm32g0xx_it.d \
./syscalls.d \
./sysmem.d \
./system_stm32g0xx.d 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


