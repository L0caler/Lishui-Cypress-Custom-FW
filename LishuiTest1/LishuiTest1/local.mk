#------------------------------------------------------------------------------
# LOCAL.MK - Allows you to customize the build/Make process for a particular 
#   PSoC Designer project.  The information you enter in this file is governed 
#   by MAKE.  It is recommended that you read about MAKE in the 
#   PSoC Designer/Documents/Supporting Documents/Make.PDF.  It is also 
#   recommended that you set the Options >> Builder >> Enable verbose
#   build messages, so that you can view the impact of the changes caused
#   by actions in this file.
#
#   There are no dependencies in the primary Make process (e.g. ...\tools\Makefile)
#   associated with this file.  Therefore, if you build the project, after saving 
#   changed information in this file, you may not get the changes you expected.
#   You can "touch" a file before you build to allow changes in this file to 
#   take effect.
#
#   Example use:
#
# To Enable the 24MHZ alignment option in ImageCraft
#CODECOMPRESSOR:=$(CODECOMPRESSOR) -z 
#
#
# To enable floating point 
#CODECOMPRESSOR:=$(CODECOMPRESSOR) -lfpm8c 
#------------------------------------------------------------------------------
# FIX: broken include path caused by a space in the PSoC Designer install path
#
# PSoC Designer is installed at "G:\Cypress\PSoC Designer\5.4". GNU make cannot
# handle spaces in paths, so psocmakemake.exe tries to convert the install path
# to its 8.3 short form before writing project.mk. 8.3 short-name creation is
# disabled on drive G: (fsutil 8dot3name query G:), so no short name exists and
# the conversion fails:
#
#   name translation failed on G:/Cypress/PSoC\ Designer/.../CypressSemiBuildMgr - 3
#   name translation failed on G:\Cypress\PSoC - 2
#   name translation failed on Designer\5.4\...\tools\include\CY8C24033 - 3
#
# psocmakemake then emits "PSOCDIR=" and "INCLUDE_PATH=" (both empty) into the
# generated project.mk, so the assembler is called with -I/tools/include and
# never finds m8c.inc. Every macro defined in m8c.inc (RAM_PROLOGUE,
# RAM_EPILOGUE, M8C_SetBank0/1, M8C_ClearWDT, ...) then reports as
# "no such mnemonic" -- those 13 errors are all fallout from the 1 missing file.
#
# project.mk is regenerated on every build (the "makemake" target runs
# psocmakemake.exe), so it cannot be patched. local.mk is read right after
# project.mk and is never regenerated, which makes it the right place to
# override both variables.
#
# G:\Cypress\PSoCDesigner is a space-free directory junction to
# "G:\Cypress\PSoC Designer", created with (no admin rights needed):
#   mklink /J G:\Cypress\PSoCDesigner "G:\Cypress\PSoC Designer"
#
PSOCDIR:=G:/Cypress/PSoCDesigner/5.4/Common/CypressSemiBuildMgr
INCLUDE_PATH:=$(PSOCDIR)/tools/include/$(BASEDEVICE)
#
# The compile/assemble rules call the bare "iccm8c.exe"/"iasm8c.exe" via PATH.
# The IDE puts "G:\Cypress\PSoC Designer\...\tools" on PATH, and iccm8c builds
# the (unquoted) path to its preprocessor icppw.exe from its own location, so
# it splits at the space:
#   extra file name "-D__IMAGECRAFT__" encountered
#   !E: cannot load source file "Designer\5.4\...\tools\icppw.exe"; file not found
# Invoking the tools through the junction path avoids that. "override" is
# required because the Makefile assigns CCMD/ASMCMD after including local.mk.
override CCMD:=$(PSOCDIR)/tools/iccm8c.exe
override ASMCMD:=$(PSOCDIR)/tools/iasm8c.exe
#
# The user sources are added here as well as in the IDE, because PSoC Designer has
# dropped them from LishuiTest1.soc before. filter-out avoids a duplicate.
USER_CSRCS:=bldc.c sense.c limit.c config.c inputs.c comm_text.c comm_binary.c
CSRCS:=$(filter-out $(USER_CSRCS),$(CSRCS)) $(USER_CSRCS)
OBJECT_SOURCES:=$(filter-out $(USER_CSRCS),$(OBJECT_SOURCES)) $(USER_CSRCS)
