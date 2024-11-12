@echo off
echo.

if "%1" == "" (
    echo Usage : %0 "Absolute Path of BinaryFile.bin"
) else (
    crt_emu_cm_redlink -flash-load-exec %1 -g -mi -2 -pQN908XC -vendor=NXP -ConnectScript=RunBootRomConnect.scp -ResetScript=qn908xreset.scp -load-base=0x00 -flash-driver=QN908XC_512K.cfx
)