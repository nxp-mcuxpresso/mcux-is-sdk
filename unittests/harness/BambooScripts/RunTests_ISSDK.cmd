@echo off
echo.
echo Parsing Command Line Arguements...

if "%1" == "" (
    echo "No Test Suite specified!"
    echo "Usage : %0 <Suite_Name> <Board_Name>"
    goto:exit_script
) else (
    set TEST_SUITE=%1
)

if "%2" == "" (
    echo "No Test Board specified!"
    echo "Usage : %0 <Suite_Name> <Board_Name>"
    goto:exit_script
) else (
    set TEST_BOARD=%2
)

if "%3" == "" (
    set BAMBOO_MODE=0
    echo No Bamboo Mode specified, using default.
) else (
    set BAMBOO_MODE=%3
)

if "%4" == "" (
    set DUT_PORT_SEQ=0
    echo No DUT COM Port Sequence no. specified, using default.
) else (
    set DUT_PORT_SEQ=%4
)

if "%5" == "" (
    set DUT_BOARD_SEQ=0
    echo No DUT Board Sequence no. specified, using default.
) else (
    set DUT_BOARD_SEQ=%5
)

if "%6" == "" (
    set SIM_PORT_SEQ=1
    echo No SIM COM Port Sequence no. specified, using default.
) else (
    set SIM_PORT_SEQ=%6
)

if "%7" == "" (
    set SIM_BOARD_SEQ=1
    echo No SIM Board Sequence no. specified, using default.
) else (
    set SIM_BOARD_SEQ=%7
)

if %BAMBOO_MODE% == 0 (
    set BAMBOO_MODE_STR=Disabled
) else (
    set BAMBOO_MODE_STR=Enabled
)

echo.
echo.
echo *******************************************************************
echo  Executing ISSDK Automated Test Suite : %TEST_SUITE%
echo *******************************************************************
echo  Test Board Name   : %TEST_BOARD%
echo  Bamboo Mode       : %BAMBOO_MODE_STR%
echo  DUT COM PORT Seq. : %DUT_PORT_SEQ%
echo  DUT BOARD Seq.    : %DUT_BOARD_SEQ%
::echo  SIM COM PORT Seq. : %SIM_PORT_SEQ%
::echo  SIM BOARD Seq.    : %SIM_BOARD_SEQ%
echo *******************************************************************

set PYTHONPATH=%CD%\unittests\harness\TestCases;%CD%\unittests\harness\TestSuites;%CD%\unittests\harness\Framework\drivers;%CD%\unittests\harness\Framework\utilities
C:\Python27\python.exe -m unittest %TEST_SUITE% %TEST_BOARD% %BAMBOO_MODE% %DUT_PORT_SEQ% %DUT_BOARD_SEQ% %SIM_PORT_SEQ% %SIM_BOARD_SEQ%

echo *******************************************************************
echo Execution for %TEST_SUITE% on FRDM-%TEST_BOARD% complete.
echo *******************************************************************
:exit_script
