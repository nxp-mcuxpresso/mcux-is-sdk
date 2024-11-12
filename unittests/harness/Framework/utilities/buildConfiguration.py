'''
@author    : B35385
@name      : buildConfiguration
@copyright : Copyright (c) 2016, NXP Semiconductors.

********************************************************************************
This is the configuration module used to store build configuration parameters.
********************************************************************************
'''

# Absolute path to directory with KEx Generated Archive Files.
KEx_ARCHIVE_PATH  = "D:\Workspace\KExArchives"

# Absolute path to directory with Example Binary Files.
BINARY_FILE_PATH  =  "D:\Workspace\Build_Temp"

# Beginning pattern and extension of Archive Files to pin point search.
FRDM_ARCHIVE_PATTERN    = "SDK_2.3_FRDM-"
LPC_ARCHIVE_PATTERN     = "SDK_2.3_LPC-"
QN_ARCHIVE_PATTERN      = "SDK_2.3_QN908xCDK-"
ARCHIVE_FILE_EXTN       = ".zip"

# Relative path to directories inside archives to look for Sensor Projects.
FRDM_SHIELD_PROJECTS_PATH = "\\boards\\frdmboard_shield\\issdk_examples\\sensors"
LPC_SHIELD_PROJECTS_PATH  = "\\boards\\lpcboard_shield\\issdk_examples\\sensors"
QN_SHIELD_PROJECTS_PATH   = "\\boards\\qn908xcdk_shield\\issdk_examples\\sensors"
FRDM_PROJECTS_PATH        = "\\boards\\frdmshield\\issdk_examples\\sensors"
