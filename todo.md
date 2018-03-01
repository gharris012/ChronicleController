# PID Tuning
- 1000 0.8 8000  
seems to hold way down well (ie: SP: 38, Ambient: 63), but continues to drive down during normal ferm (ie: SP: 68, Ambient: 63)
- 1000 100 1  
decent, but tends to overshoot down - does not shut down fast enough
- 5000 20 100  
  - 7/29/2017 - works well enough, but doesn't quite settle

# Private
export PARTICLE_DEVICE=<deviceId>  
export PARTICLE_APPDIR=/c/Code/particle/projects/ChronicleController  


# Program Flow
### Thread :: Temp Sensors
- Periodically update temperature
  #### DS Temp Sensors
  - begin conversion
  - wait
  - read
  - store

  #### Thermistors
  - read
  - store

### Thread :: PID
- Periodically run
  - check window
  - compute PID
  - set window

### Thread :: Control
- Check window
  - on/off


# Sample Run

control->heat_pid.Compute(); 
control->


# Add Heater
(threshold=2)
set + threshold => start chiller PID
set - threshold => start heater PID

# VS Code Setup

## Environment Variables:
GNUARMNONEHOME
Location of GNU ARM None (ie: C:\Program Files (x86)\GNU Tools ARM Embedded\5.3 2016q1)
GNUARMNONEVERSION
Compiler version (ie: 5.3.1 (from C:\Program Files (x86)\GNU Tools ARM Embedded\5.3 2016q1\lib\gcc\arm-none-eabi\5.3.1)
PARTICLEHOME
Location of particle firmware source (ie: C:\Code\src\particle)

## Defines
see  
http://shadetail.com/blog/using-visual-studio-code-for-arm-development-defines/

echo | arm-none-eabi-gcc -dM -E - > gcc-defines.txt

delete all `#define`  
(in vscode, highlight `#define`, ctrl-shift-l (select all occurances), delete)

use find/replace to escape all quotes:  
`__VERSION__ \"5.4.1 20160919 (release) [ARM/embedded-5-branch revision 240496]\"`

use regex find/replace to transform  
`__ATOMIC_CONSUME 1` to `"__ATOMIC_CONSUME=1"`  
find: `^([^ ]*) (.*)$`  
replace: `"$1=$2",`

clean up the extra comma at the end, and paste into `defines` section of `c_cpp_properties.json`.

## Includes

### GNU/System
http://shadetail.com/blog/using-visual-studio-code-for-arm-development-include-paths/  

### Particle
In PARTICLEHOME
```dir /ON /AD /S /B | grep -v tests | grep -v .git | grep -v build | grep -v docs | sed s/\\/\//g | sed 's/.*/"&",/g' > out.txt```

change all `PARTICLEHOME` to `${env:PARTICLEHOME}

clean up the extra comma at the end, and paste into `includePath` section of `c_cpp_properties.json`.
