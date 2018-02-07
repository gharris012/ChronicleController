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
