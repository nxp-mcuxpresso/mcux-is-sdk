@echo off

cls

IarBuild.exe FRDM_K64F_AGM01_FreeRTOSTwoTasks/main.ewp -clean Debug
IarBuild.exe  FRDM_K64F_MULT2B_FreeRTOSTwoTasksSPI/main.ewp -clean Debug
IarBuild.exe  FRDM_K64F_MULT2B_FreeRTOSTwoTasksPowerCycling/main.ewp -clean Debug
IarBuild.exe  FRDM_K64F_MULT2B_FreeRTOSTwoTasks/main.ewp -clean Debug
IarBuild.exe  FRDM_K64F_MULT2B_BARE_METAL/main.ewp -clean Debug
IarBuild.exe  FRDM_K64F_AGM02/main.ewp -clean Debug
IarBuild.exe  FRDM_K22F_AGM01_BARE_METAL/main.ewp -clean Debug
IarBuild.exe  FRDM_K22F_AGM01_FreeRTOSTwoTasks/main.ewp -clean Debug
IarBuild.exe  FRDM_K22F_MULT2B_FreeRTOSTwoTasks/main.ewp -clean Debug
IarBuild.exe   FRDM_K64F_AGM02_PowerCycling/main.ewp -clean Debug
IarBuild.exe  FRDM_K64F_MULT2B_FreeRTOSTwoTasks8471I2C/main.ewp -clean Debug
IarBuild.exe  FRDM_K64F_MULT2B_FreeRTOSTwoTasksPowerCycling/main.ewp -clean Debug
