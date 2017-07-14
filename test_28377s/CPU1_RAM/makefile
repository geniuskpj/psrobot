################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

CG_TOOL_ROOT := C:/ti/ccsv5/tools/compiler/c2000_6.2.11

ORDERED_OBJS += \
$(GEN_CMDS__FLAG) \
"./ustdlib.obj" \
"./serial.obj" \
"./robotis.obj" \
"./pwmsetup.obj" \
"./mmc-F2837x.obj" \
"./kpj485_mod_ps .obj" \
"./interrupt.obj" \
"./ff.obj" \
"./cmdline.obj" \
"./F2837xD_usDelay.obj" \
"./F2837xD_SysCtrl.obj" \
"./F2837xD_Sci.obj" \
"./F2837xD_PieVect.obj" \
"./F2837xD_PieCtrl.obj" \
"./F2837xD_Gpio.obj" \
"./F2837xD_GlobalVariableDefs.obj" \
"./F2837xD_EPwm.obj" \
"./F2837xD_DefaultISR.obj" \
"./F2837xD_CpuTimers.obj" \
"./F2837xD_CodeStartBranch.obj" \
"./F2837xD_Adc.obj" \
-l"2837xD_RAM_lnk_cpu1.cmd" \
-l"F2837xD_Headers_nonBIOS_cpu1.cmd" \
-l"libc.a" \

-include ../makefile.init

RM := DEL /F
RMDIR := RMDIR /S/Q

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir_vars.mk
-include subdir_rules.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(S62_DEPS)),)
-include $(S62_DEPS)
endif
ifneq ($(strip $(C64_DEPS)),)
-include $(C64_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(S55_DEPS)),)
-include $(S55_DEPS)
endif
ifneq ($(strip $(C67_DEPS)),)
-include $(C67_DEPS)
endif
ifneq ($(strip $(C??_DEPS)),)
-include $(C??_DEPS)
endif
ifneq ($(strip $(CLA_DEPS)),)
-include $(CLA_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(S??_DEPS)),)
-include $(S??_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(C62_DEPS)),)
-include $(C62_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(ASM_UPPER_DEPS)),)
-include $(ASM_UPPER_DEPS)
endif
ifneq ($(strip $(K_DEPS)),)
-include $(K_DEPS)
endif
ifneq ($(strip $(C43_DEPS)),)
-include $(C43_DEPS)
endif
ifneq ($(strip $(S67_DEPS)),)
-include $(S67_DEPS)
endif
ifneq ($(strip $(SA_DEPS)),)
-include $(SA_DEPS)
endif
ifneq ($(strip $(S43_DEPS)),)
-include $(S43_DEPS)
endif
ifneq ($(strip $(OPT_DEPS)),)
-include $(OPT_DEPS)
endif
ifneq ($(strip $(S64_DEPS)),)
-include $(S64_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(C55_DEPS)),)
-include $(C55_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: ps_1215.out

# Tool invocations
ps_1215.out: $(OBJS) $(CMD_SRCS) $(GEN_CMDS)
	@echo 'Building target: $@'
	@echo 'Invoking: C2000 Linker'
	"C:/ti/ccsv5/tools/compiler/c2000_6.2.11/bin/cl2000" -v28 -ml -mt --float_support=fpu32 -g --define=CPU1 --diag_warning=225 --display_error_number --printf_support=nofloat -z -m"ps_1215.map" --heap_size=0x1000 --stack_size=0x1000 --warn_sections -i"C:/ti/ccsv5/tools/compiler/c2000_6.2.11/lib" -i"C:/Users/ps/Documents/dspcode/cmd" -i"C:/ti/ccsv5/tools/compiler/c2000_6.2.11/include" --reread_libs --display_error_number --xml_link_info="ps_1215_linkInfo.xml" --entry_point=code_start --rom_model -o "ps_1215.out" $(ORDERED_OBJS)
	@echo 'Finished building target: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(C2000_EXECUTABLE_OUTPUTS__QUOTED) "ps_1215.out"
	-$(RM) "F2837xD_Adc.pp" "F2837xD_CpuTimers.pp" "F2837xD_DefaultISR.pp" "F2837xD_EPwm.pp" "F2837xD_GlobalVariableDefs.pp" "F2837xD_Gpio.pp" "F2837xD_PieCtrl.pp" "F2837xD_PieVect.pp" "F2837xD_Sci.pp" "F2837xD_SysCtrl.pp" "cmdline.pp" "ff.pp" "interrupt.pp" "kpj485_mod_ps .pp" "mmc-F2837x.pp" "pwmsetup.pp" "robotis.pp" "serial.pp" "ustdlib.pp" 
	-$(RM) "F2837xD_Adc.obj" "F2837xD_CodeStartBranch.obj" "F2837xD_CpuTimers.obj" "F2837xD_DefaultISR.obj" "F2837xD_EPwm.obj" "F2837xD_GlobalVariableDefs.obj" "F2837xD_Gpio.obj" "F2837xD_PieCtrl.obj" "F2837xD_PieVect.obj" "F2837xD_Sci.obj" "F2837xD_SysCtrl.obj" "F2837xD_usDelay.obj" "cmdline.obj" "ff.obj" "interrupt.obj" "kpj485_mod_ps .obj" "mmc-F2837x.obj" "pwmsetup.obj" "robotis.obj" "serial.obj" "ustdlib.obj" 
	-$(RM) "F2837xD_CodeStartBranch.pp" "F2837xD_usDelay.pp" 
	-@echo 'Finished clean'
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets

