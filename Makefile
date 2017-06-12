nRF52_SDK		= 	/home/konrad/Eclipse_Workspace/LIBS/nRF/nRF52_SDK
S132_HEX 		= 	/home/konrad/Eclipse_Workspace/LIBS/nRF/SoftDevice/s132_nrf52_3.1.0_softdevice.hex

include $(nRF52_SDK)/sdk.mk

CC = arm-none-eabi-gcc
OPTIMIZATION = -O0
BUILD_FOLDER = Build_Output
OUTPUT_BINARY_FOLDER = $(BUILD_FOLDER)
OUTPUT_BINARY_NAME = Car_Tracker
LINKER_SCRIPT = armgcc_s132_nrf52832_xxaa.ld
LINKER_COMMON_SCRIPT = nrf5x_common.ld

#------------------------ INCLUDE PATHS --------------------------------

INC_PATHS += -Iinc/
INC_PATHS += -Ihardware/
INC_PATHS += $(SDK_INCLUDE_PATHS) 

#----------------------- COMPILING FLAGS ------------------------------

CFLAGS +=	$(OPTIMIZATION)
CFLAGS += 	-mcpu=cortex-m4
CFLAGS +=   -mabi=aapcs
CFLAGS +=   -mthumb
CFLAGS += 	-mfloat-abi=soft
CFLAGS += 	-mfpu=fpv4-sp-d16
CFLAGS +=   -DARM_MATH_CM4
CFLAGS +=	--std=gnu99
CFLAGS += 	-Werror
CFLAGS +=	-fmessage-length=0
CFLAGS +=	-ffunction-sections
CFLAGS +=	-fdata-sections
CFLAGS += 	-fno-strict-aliasing
CFLAGS +=	-g
CFLAGS += 	-fno-builtin --short-enums
#CFLAGS += 	-fomit-frame-pointer
CFLAGS +=	-DNRF52832_XXAA
CFLAGS += 	-DNRF52832


ASMFLAGS += -x assembler-with-cpp
ASMFLAGS += -DARM_MATH_CM4
ASMFLAGS += -mfloat-abi=hard
ASMFLAGS += -mfpu=fpv4-sp-d16
#------------------------ LINKER FLAGS --------------------------------

LDFLAGS += -mcpu=cortex-m4
LDFLAGS += -mabi=aapcs
LDFLAGS += -mfloat-abi=soft
LDFLAGS += -mfpu=fpv4-sp-d16
LDFLAGS += -Xlinker -Map=$(OUTPUT_BINARY_FOLDER)/$(OUTPUT_BINARY_NAME).map
LDFLAGS += -Wl,--gc-sections
LDFLAGS += --specs=nano.specs -lc 
LDFLAGS += --specs=nosys.specs
LDFLAGS += -T"$(nRF52_SDK)/$(LINKER_SCRIPT)"
LDFLAGS += -L"$(nRF52_SDK)"
#----------------------- PROJECT SOURCES ------------------------------

ASM_SOURCE_FILES += gcc_startup_nrf52.s

C_SOURCE_FILES = src/main.c
C_SOURCE_FILES += src/system_nrf52.c
C_SOURCE_FILES += hardware/Systick.c

#------------------------ COMPILATION VARIABLES ------------------------

# List of source file names without directories
C_SOURCE_FILE_NAMES = $(notdir $(C_SOURCE_FILES))

# List of paths where the source files lie
C_PATHS = $(dir $(C_SOURCE_FILES))

# List of object files prefixed with build folder where the objects file will be held
C_OBJECTS = $(addprefix $(BUILD_FOLDER)/, $(C_SOURCE_FILE_NAMES:.c=.o))

ASM_SOURCE_FILE_NAMES = $(notdir $(ASM_SOURCE_FILES))

ASM_PATHS = $(dir $(ASM_SOURCE_FILES))

ASM_OBJECTS = $(addprefix $(BUILD_FOLDER)/, $(ASM_SOURCE_FILE_NAMES:.s=.o))

# List of paths where the sources are to be searched for
vpath %.c $(C_PATHS)
vpath %.S $(ASM_PATHS)

OBJECTS = $(C_OBJECTS) $(ASM_OBJECTS)

#------------------------------- TARGETS -------------------------------

# Working target to compile a single file - called for each of the .c file
$(BUILD_FOLDER)/%.o: %.c
	@echo "Compiling file: $<"
	$(CC) $(CFLAGS) $(INC_PATHS) -c $< -o $@
	
# Assemble files
$(BUILD_FOLDER)/%.o: %.s
	@echo Compiling ASM file: $(notdir $<)
	$(CC) $(ASMFLAGS) $(INC_PATHS) -c -o $@ $<

# Working target for linking compiled files
$(OUTPUT_BINARY_FOLDER)/$(OUTPUT_BINARY_NAME).elf: $(OBJECTS)
	$(info info: ------- GENERATING ELF FILE -------)
	@echo "Linking files..."
	$(CC) $(LDFLAGS) $(OBJECTS) -o $@
	arm-none-eabi-objcopy -O ihex "$@" "$(OUTPUT_BINARY_FOLDER)/$(OUTPUT_BINARY_NAME).hex"
	arm-none-eabi-objcopy -O binary "$@" "$(OUTPUT_BINARY_FOLDER)/$(OUTPUT_BINARY_NAME).bin"
	@echo "Finished linking..."
	
# Target for main compilation
all: print_start_info $(BUILD_FOLDER) $(OUTPUT_BINARY_FOLDER)/$(OUTPUT_BINARY_NAME).elf
	@echo "Finished building: $@"
	@echo " "

# Target for cleaning the directory
clean:
	rm -rf $(BUILD_FOLDER)
	@echo "Build directory cleaned."
	
# Target for logging compilation start
print_start_info:
	$(info info: -------- COMPILING FILES ---------)
	@echo "Starting building with optimization: $(OPTIMIZATION)"
	
# Target for creating build folder
$(BUILD_FOLDER):
	@mkdir $(BUILD_FOLDER)
	