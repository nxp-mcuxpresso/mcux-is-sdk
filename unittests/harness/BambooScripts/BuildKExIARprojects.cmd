@echo off
echo.
echo *******************************************************************
echo Executing IAR Project Builder Script...
echo *******************************************************************

C:\Python27\python.exe %CD%\unittests\harness\Framework\utilities\buildIARprojects.py

if %ERRORLEVEL% EQU 0 (
    set RESULT_STRING=Complete
) else (
    set RESULT_STRING=Failed
)

echo *******************************************************************
echo IAR Project Builder script execution %RESULT_STRING%.
echo *******************************************************************
