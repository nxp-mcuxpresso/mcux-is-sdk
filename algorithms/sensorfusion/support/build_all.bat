@echo off

cls

echo Press return to build FRDM_K64F_AGM01_FreeRTOSTwoTasks
set /p input=""
IarBuild.exe FRDM_K64F_AGM01_FreeRTOSTwoTasks/main.ewp Debug

echo Press return to build FRDM_K64F_MULT2B_FreeRTOSTwoTasksSPI
set /p input=""
IarBuild.exe  FRDM_K64F_MULT2B_FreeRTOSTwoTasksSPI/main.ewp Debug

echo Press return to build FRDM_K64F_MULT2B_FreeRTOSTwoTasksPowerCycling
set /p input=""
IarBuild.exe  FRDM_K64F_MULT2B_FreeRTOSTwoTasksPowerCycling/main.ewp Debug

echo Press return to build FRDM_K64F_MULT2B_FreeRTOSTwoTasks
set /p input=""
IarBuild.exe  FRDM_K64F_MULT2B_FreeRTOSTwoTasks/main.ewp Debug

echo Press return to build FRDM_K64F_MULT2B_BARE_METAL
set /p input=""
IarBuild.exe  FRDM_K64F_MULT2B_BARE_METAL/main.ewp Debug

echo Press return to build FRDM_K64F_AGM02
set /p input=""
IarBuild.exe  FRDM_K64F_AGM02/main.ewp Debug

echo Press return to build FRDM_K22F_AGM01_BARE_METAL
set /p input=""
IarBuild.exe  FRDM_K22F_AGM01_BARE_METAL/main.ewp Debug

echo Press return to build FRDM_K22F_AGM01_FreeRTOSTwoTasks
set /p input=""
IarBuild.exe  FRDM_K22F_AGM01_FreeRTOSTwoTasks/main.ewp Debug

echo Press return to build FRDM_K22F_MULT2B_FreeRTOSTwoTasks
set /p input=""
IarBuild.exe  FRDM_K22F_MULT2B_FreeRTOSTwoTasks/main.ewp Debug

echo Press return to build FRDM_K64F_AGM02_PowerCycling
set /p input=""
IarBuild.exe   FRDM_K64F_AGM02_PowerCycling/main.ewp Debug

echo Press return to build FRDM_K64F_MULT2B_FreeRTOSTwoTasks8471I2C
set /p input=""
IarBuild.exe  FRDM_K64F_MULT2B_FreeRTOSTwoTasks8471I2C/main.ewp Debug

echo Press return to build FRDM_K64F_MULT2B_FreeRTOSTwoTasksPowerCycling
set /p input=""
IarBuild.exe  FRDM_K64F_MULT2B_FreeRTOSTwoTasksPowerCycling/main.ewp Debug
