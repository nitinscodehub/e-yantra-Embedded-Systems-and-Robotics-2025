################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL := cmd.exe
RM := rm -rf

USER_OBJS :=

LIBS := 
PROJ := 

O_SRCS := 
C_SRCS := 
S_SRCS := 
S_UPPER_SRCS := 
OBJ_SRCS := 
ASM_SRCS := 
PREPROCESSING_SRCS := 
OBJS := 
OBJS_AS_ARGS := 
C_DEPS := 
C_DEPS_AS_ARGS := 
EXECUTABLES := 
OUTPUT_FILE_PATH :=
OUTPUT_FILE_PATH_AS_ARGS :=
AVR_APP_PATH :=$$$AVR_APP_PATH$$$
QUOTE := "
ADDITIONAL_DEPENDENCIES:=
OUTPUT_FILE_DEP:=
LIB_DEP:=
LINKER_SCRIPT_DEP:=

# Every subdirectory with source files must be described here
SUBDIRS := 


# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS +=  \
../9_ADC_Sensor_Display_on_LCD.c \
../lcd.c


PREPROCESSING_SRCS += 


ASM_SRCS += 


OBJS +=  \
9_ADC_Sensor_Display_on_LCD.o \
lcd.o

OBJS_AS_ARGS +=  \
9_ADC_Sensor_Display_on_LCD.o \
lcd.o

C_DEPS +=  \
9_ADC_Sensor_Display_on_LCD.d \
lcd.d

C_DEPS_AS_ARGS +=  \
9_ADC_Sensor_Display_on_LCD.d \
lcd.d

OUTPUT_FILE_PATH +=9_ADC_Sensor_Display_on_LCD.elf

OUTPUT_FILE_PATH_AS_ARGS +=9_ADC_Sensor_Display_on_LCD.elf

ADDITIONAL_DEPENDENCIES:=

OUTPUT_FILE_DEP:= ./makedep.mk

LIB_DEP+= 

LINKER_SCRIPT_DEP+= 


# AVR32/GNU C Compiler
./9_ADC_Sensor_Display_on_LCD.o: .././9_ADC_Sensor_Display_on_LCD.c
	@echo Building file: $<
	@echo Invoking: AVR/GNU C Compiler : 5.4.0
	$(QUOTE)D:\Atmel_Studio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-gcc.exe$(QUOTE)  -x c -funsigned-char -funsigned-bitfields -DDEBUG  -I"D:\Atmel_Studio\7.0\Packs\atmel\ATmega_DFP\1.3.300\include"  -O1 -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -mrelax -g2 -Wall -mmcu=atmega2560 -B "D:\Atmel_Studio\7.0\Packs\atmel\ATmega_DFP\1.3.300\gcc\dev\atmega2560" -c -std=gnu99 -MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)"   -o "$@" "$<" 
	@echo Finished building: $<
	

./lcd.o: .././lcd.c
	@echo Building file: $<
	@echo Invoking: AVR/GNU C Compiler : 5.4.0
	$(QUOTE)D:\Atmel_Studio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-gcc.exe$(QUOTE)  -x c -funsigned-char -funsigned-bitfields -DDEBUG  -I"D:\Atmel_Studio\7.0\Packs\atmel\ATmega_DFP\1.3.300\include"  -O1 -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -mrelax -g2 -Wall -mmcu=atmega2560 -B "D:\Atmel_Studio\7.0\Packs\atmel\ATmega_DFP\1.3.300\gcc\dev\atmega2560" -c -std=gnu99 -MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)"   -o "$@" "$<" 
	@echo Finished building: $<
	




# AVR32/GNU Preprocessing Assembler



# AVR32/GNU Assembler




ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: $(OUTPUT_FILE_PATH) $(ADDITIONAL_DEPENDENCIES)

$(OUTPUT_FILE_PATH): $(OBJS) $(USER_OBJS) $(OUTPUT_FILE_DEP) $(LIB_DEP) $(LINKER_SCRIPT_DEP)
	@echo Building target: $@
	@echo Invoking: AVR/GNU Linker : 5.4.0
	$(QUOTE)D:\Atmel_Studio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-gcc.exe$(QUOTE) -o$(OUTPUT_FILE_PATH_AS_ARGS) $(OBJS_AS_ARGS) $(USER_OBJS) $(LIBS) -Wl,-Map="9_ADC_Sensor_Display_on_LCD.map" -Wl,--start-group -Wl,-lm  -Wl,--end-group -Wl,--gc-sections -mrelax -mmcu=atmega2560 -B "D:\Atmel_Studio\7.0\Packs\atmel\ATmega_DFP\1.3.300\gcc\dev\atmega2560"  
	@echo Finished building target: $@
	"D:\Atmel_Studio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-objcopy.exe" -O ihex -R .eeprom -R .fuse -R .lock -R .signature -R .user_signatures  "9_ADC_Sensor_Display_on_LCD.elf" "9_ADC_Sensor_Display_on_LCD.hex"
	"D:\Atmel_Studio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-objcopy.exe" -j .eeprom  --set-section-flags=.eeprom=alloc,load --change-section-lma .eeprom=0  --no-change-warnings -O ihex "9_ADC_Sensor_Display_on_LCD.elf" "9_ADC_Sensor_Display_on_LCD.eep" || exit 0
	"D:\Atmel_Studio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-objdump.exe" -h -S "9_ADC_Sensor_Display_on_LCD.elf" > "9_ADC_Sensor_Display_on_LCD.lss"
	"D:\Atmel_Studio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-objcopy.exe" -O srec -R .eeprom -R .fuse -R .lock -R .signature -R .user_signatures "9_ADC_Sensor_Display_on_LCD.elf" "9_ADC_Sensor_Display_on_LCD.srec"
	"D:\Atmel_Studio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-size.exe" "9_ADC_Sensor_Display_on_LCD.elf"
	
	





# Other Targets
clean:
	-$(RM) $(OBJS_AS_ARGS) $(EXECUTABLES)  
	-$(RM) $(C_DEPS_AS_ARGS)   
	rm -rf "9_ADC_Sensor_Display_on_LCD.elf" "9_ADC_Sensor_Display_on_LCD.a" "9_ADC_Sensor_Display_on_LCD.hex" "9_ADC_Sensor_Display_on_LCD.lss" "9_ADC_Sensor_Display_on_LCD.eep" "9_ADC_Sensor_Display_on_LCD.map" "9_ADC_Sensor_Display_on_LCD.srec" "9_ADC_Sensor_Display_on_LCD.usersignatures"
	