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
../1_Buzzer_Beep.c


PREPROCESSING_SRCS += 


ASM_SRCS += 


OBJS +=  \
1_Buzzer_Beep.o

OBJS_AS_ARGS +=  \
1_Buzzer_Beep.o

C_DEPS +=  \
1_Buzzer_Beep.d

C_DEPS_AS_ARGS +=  \
1_Buzzer_Beep.d

OUTPUT_FILE_PATH +=1_Buzzer_Beep.elf

OUTPUT_FILE_PATH_AS_ARGS +=1_Buzzer_Beep.elf

ADDITIONAL_DEPENDENCIES:=

OUTPUT_FILE_DEP:= ./makedep.mk

LIB_DEP+= 

LINKER_SCRIPT_DEP+= 


# AVR32/GNU C Compiler
./1_Buzzer_Beep.o: .././1_Buzzer_Beep.c
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
	$(QUOTE)D:\Atmel_Studio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-gcc.exe$(QUOTE) -o$(OUTPUT_FILE_PATH_AS_ARGS) $(OBJS_AS_ARGS) $(USER_OBJS) $(LIBS) -Wl,-Map="1_Buzzer_Beep.map" -Wl,--start-group -Wl,-lm  -Wl,--end-group -Wl,--gc-sections -mrelax -mmcu=atmega2560 -B "D:\Atmel_Studio\7.0\Packs\atmel\ATmega_DFP\1.3.300\gcc\dev\atmega2560"  
	@echo Finished building target: $@
	"D:\Atmel_Studio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-objcopy.exe" -O ihex -R .eeprom -R .fuse -R .lock -R .signature -R .user_signatures  "1_Buzzer_Beep.elf" "1_Buzzer_Beep.hex"
	"D:\Atmel_Studio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-objcopy.exe" -j .eeprom  --set-section-flags=.eeprom=alloc,load --change-section-lma .eeprom=0  --no-change-warnings -O ihex "1_Buzzer_Beep.elf" "1_Buzzer_Beep.eep" || exit 0
	"D:\Atmel_Studio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-objdump.exe" -h -S "1_Buzzer_Beep.elf" > "1_Buzzer_Beep.lss"
	"D:\Atmel_Studio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-objcopy.exe" -O srec -R .eeprom -R .fuse -R .lock -R .signature -R .user_signatures "1_Buzzer_Beep.elf" "1_Buzzer_Beep.srec"
	"D:\Atmel_Studio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-size.exe" "1_Buzzer_Beep.elf"
	
	





# Other Targets
clean:
	-$(RM) $(OBJS_AS_ARGS) $(EXECUTABLES)  
	-$(RM) $(C_DEPS_AS_ARGS)   
	rm -rf "1_Buzzer_Beep.elf" "1_Buzzer_Beep.a" "1_Buzzer_Beep.hex" "1_Buzzer_Beep.lss" "1_Buzzer_Beep.eep" "1_Buzzer_Beep.map" "1_Buzzer_Beep.srec" "1_Buzzer_Beep.usersignatures"
	