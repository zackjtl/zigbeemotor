# zstack
On the coordinator
1. Two brushless motor are attached on.
2. Send break & enable signal to the relays with the wire connect to COM.
3. Send PWM wave to a transistor switch, adapt the 3.3v signal to the motor controller's PWM port.
4. 
 
On the end device
1. Grab the alaog output from a 5-way joystick by the on-chip a/d converter.
2. A timer is routinely grabbing the joystick value, and sending the value to coordinator.
 
