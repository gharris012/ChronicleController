
setlocal
set oldpwd=%CD%

cd c:\Code\particle\firmware

make %1 PLATFORM=photon APPDIR=/c/Code/particle/projects/ChronicleController

cd %oldpwd%

endlocal