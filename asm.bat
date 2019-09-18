@ECHO OFF
REM Usage: asm.bat program.asm
if %1asdf==asdf goto usage
customasm.exe -i instruction-set.txt -o ram.hex -f hexstr2 %1
REM copy ram.hex ..\computer32.2
goto end
:usage
customasm.exe -i instruction-set.txt -o ram.hex -f hexstr2 test_sdram.asm
:end
pause
