# brickpi functions

BP.set_motor_power(BP.PORT_A, power)
- sets raw power level (-100, 100) which makes motor move without PID control

BP.set_motor_power(BP.PORT_A, BP.MOTOR_FLOAT)
- set motor to 'float' without power --> it can be tuned by hand
- encoder can still be read --> can interface with the robot by hand tuning

BP.get_motor_encoder(BP.PORT_A)
- returns current encoder position in degrees

BP.offset_motor_encoder(BP.PORT_A, BP.get_motor_encoder(BP.PORT_A))
- resets encoder count to zero

BP.set_motor_position(BP.PORT_A, degrees)
- sets position demand for the motor in degrees, and starts PID control to reach it

BP.set_motor_dps(BP.PORT_A, dps)
- sets velocity demand for the motor in degrees per second, starts PID control to reach it

BP.get_motor_status(BP.PORT_A)
- returns (current status flag, power in percent, encoder position (degrees), current velocity (dps))

BP.set_motor_limits(BP.PORT_A, power, dps)
- sets limits on power and dps that will be used in PID control
- useful to protect BrickPi from overloading
- typically < 70% power

BP.set_motor_position_kp(BP.PORT_A, kp)
- set PID proportional gain constant
- default is 25

BP.reset_all()


notes:
- if you want your program to wait till a PID action is completed, you need to
  run a loop to monitor the motor and check for the finish condition (also use
  a threshold for the loop test because the motor may never reach the exact
  position you request)
- if your program exits without stopping the motors then they will keep moving
  --> try: except KeyboardInterrupt: reset_all()



# basic movement
```
def move(left_offset, right_offset):
  left_curr_pos = BP.get_motor_encoder(BP.PORT_A)
  right_curr_pos = BP.get_motor_encoder(BP.PORT_D)
  left_target_pos = left_curr_pos + left_offset
  right_target_pos = right_curr_pos + right_offset
  while (abs(BP.get_motor_encoder(BP.PORT_A) - left_target_pos) > 10 and
         abs(BP.get_motor_encoder(BP.PORT_D) - right_target_pos) > 10):
    BP.set_motor_position(BP.PORT_A, left_target_pos)
    BP.set_motor_position(BP.PORT_D, right_target_pos)

def rotate(angle_degeres):
  offset = ROTATE_CALIBRATION * angle_degrees
  self.move(offset, -offset)

def translate(distance_cm):
  offset = TRANSLATE_CALIBRATION * distance_cm
  self.move(offset, offset)
```

# servoing



# wall following
wall is on the left side of the robot. higher error --> right wheel moves faster --> move closer to the wall --> reduce error
```
K = 3  # arbitrary constant
REQUIRED_DISTANCE_FROM_WALL = 20

try:
  while True:
    try:
      distance_cm = BP.get_sensor(BP.PORT_C)
      speed = 200
      error = K * (distance_cm - REQUIRED_DISTANCE_FROM_WALL)
      BP.set_motor_dps(BP.PORT_A, speed - error)  # left wheel
      BP.set_motor_dps(BP.PORT_D, speed + error)  # right wheel
    except brickpi3.SensorError as error:
      print(error)
except KeyboardInterrupt:
  pass
finally:
  BP.reset_all()
```

# occupancy mapping



# mcl




# signature mapping
