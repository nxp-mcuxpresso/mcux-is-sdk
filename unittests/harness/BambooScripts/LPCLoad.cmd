@echo off
echo.

if "%1" == "" (
    echo Usage : %0 "Absolute Path of BinaryFile.bin"
) else (
    crt_emu_cm_redlink -flash-load-exec %1 -g -2  -vendor=NXP -pLPC54114J256 -load-base=0x00    -reset=vectreset -flash-driver=LPC5411x_256K.cfx
)