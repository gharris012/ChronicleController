
setlocal
set oldpwd=%CD%

cd c:\Code\particle\firmware
set projectdir=%2
set "inappdir=%projectdir:\=/%"

make %1 PLATFORM=photon APPDIR=%inappdir%

cd %oldpwd%

endlocal