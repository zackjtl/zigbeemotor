# zstack
On the coordinator
1. Recieve action command (with specific string) from end device.
2. Two brushless motor controllers are attached on.
3. Send break & enable signal to the relays with the wire connect to controllers' COM.
4. Send PWM wave to a transistor switch, adapt the 3.3v signal to the motor controllers' PWM port.
 
On the end device
1. Grab the alaog output from a 5-way joystick by the on-chip a/d converter.
2. A timer is routinely grabbing the joystick value, and sending the value to coordinator.
 
