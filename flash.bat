cd %~dp0
set path=%path%;C:\Program Files\SEGGER\JLinkARM_V440

:start

@choice /c:se /m SCons,Exit

@if errorlevel 2 goto Exit
@if errorlevel 1 goto SCons

:SCons
jlink jlink_flash.txt
goto start

:Exit