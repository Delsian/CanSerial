# CanSerial

Serial port emulation over CAN bus.
Working on Raspberry Pi with MCP2515 module.
Main goal - emulate serial port to use tiny external modules together with
Klipper controller for 3D-printers https://github.com/KevinOConnor/klipper

Adding CAN bus to Raspberry Pi completely described here
https://www.raspberrypi.org/forums/viewtopic.php?f=44&t=141052

## Protocol

Only Base Frame format (CAN 2.0 B) implemented.
Master node periodically sends frame with ID 0x321 and zero length as UUID request.
Slave node responds to this frame with ID 0x323, containing 6 bytes of unique serial number.
Master will check local dictionary and responds with ID 0x322, containig port number.
After receiving port number slave node should ignore next UUID requests and accepts only 
frames with id 0x17F+(2 x PortNumber) 

After assigning port to slave, Emulator creates /tmp/ttyCANxx node emulating serial port.

## Build procedure

```
git clone git@github.com:Delsian/CanSerial.git
cd CanSerial
cmake .
make
./canserial
```


If you get "Socket init error: xxx" - check your CAN configuration.

