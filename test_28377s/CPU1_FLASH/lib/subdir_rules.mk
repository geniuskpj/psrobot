################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
lib/mklib.obj: ../lib/mklib.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv5/tools/compiler/c2000_6.2.11/bin/cl2000" -v28 -ml -mt --include_path="C:/Users/ps/Documents/dspcode/test_28377s/include" -g --define=CPU1 --define=_FLASH --diag_warning=225 --display_error_number --preproc_with_compile --preproc_dependency="lib/mklib.pp" --obj_directory="lib" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '


