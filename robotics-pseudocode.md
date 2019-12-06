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
  if angle_degrees > 180:
    angle_degrees = 180 - angle_degrees  # set it to turn anticlockwise
  offset = ROTATE_CALIBRATION * angle_degrees
  self.move(offset, -offset)

def translate(distance_cm):
  offset = TRANSLATE_CALIBRATION * distance_cm
  self.move(offset, offset)
```

# servoing
```
while True:
  try:
    distance_cm = BP.get_sensor(BP.PORT_C)
    error = K * (distance_cm - REQUIRED_DISTANCE_FROM_WALL)
    BP.set_motor_dps(BP.PORT_A, error)
    BP.set_motor_dps(BP.PORT_D, error)
    time.sleep(0.02)
  except brickpi3.SensorError as error:
    print(error)
```


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

# waypoint navigation
```
NUM_PARTICLES = 100
particles = [(0, 0, 0) for i in range(NUM_PARTICLES)]
curr_position_estimate = (0, 0, 0)

def update_particles(is_translate, point, offset):
  if is_translate:
    for i in range(NUM_PARTICLES):
      new_particle = self.update_translate(point, offset)
      self.particles[i] = new_particle
  else:
    for i in range(NUM_PARTICLES):
      new_particle = self.update_rotate(point, offset)
      self.particles[i] = new_particle
  new_estimate = [0, 0, 0]
  for i in range(NUM_PARTICLES):
    new_estimate[0] += self.weights[i] * self.particles[i][0]
    new_estimate[1] += self.weights[i] * self.particles[i][1]
    new_estimate[2] += self.weights[i] * self.particles[i][2]
  self.curr_position_estimate = tuple(new_estimate)

def update_rotate(point, angle_degrees):
  new_angle = point[2] + angle_degeres + random.gauss(0, g)
  new_point = (point[0], point[1], new_angle)
  return new_point

def update_translate(point, distance):
  new_x = point[0] + (distance + random.gauss(0, e)) * math.cos(point[2])
  new_y = point[1] + (distance + random.gauss(0, e)) * math.sin(point[2])
  new_angle = point[2] + random.gauss(0, f)
  new_point = (new_x, new_y, new_angle)
  return new_point

def navigate_to_waypoint(x, y):
  x_diff = x - curr_position_estimate[0]
  y_diff = y - curr_position_estimate[1]
  distance = math.sqrt(x_diff**2, y_diff**2)
  angle_diff = math.degrees(math.atan2(y_diff, x_diff))
  angle = angle_diff - curr_position_estimate[3]

  # scale it to be 0 < angle < 360
  if angle > 360:
    angle = angle - 360
  if angle < 0:
    angle = angle + 360

  self.rotate(angle)
  self.translate(distance)

```

# mcl
```
def calculate_likelihood(x, y, theta, z):
  # find out which wall the sonar beam will hit if the robot is at position (x, y, theta)
  walls_hit = []  # (m, wall)
  for (ax, ay, bx, by) in walls:
    numerator = (by - ay)*(ax - x) - (bx - ax)*(ay - y)
    denominator = (by - ay)*math.cos(theta) - (bx - ax)*math.sin(theta)
    m = numerator / denominator

    # check if the sonar will hit within limits of the wall
    x_pos = x + m * math.cos(theta)
    y_pos = y + m * math.sin(theta)

    if x_pos >= ax and x_pos <= bx:
      walls_hit.append(m, (ax, ay, bx, by))

  # get closest wall
  m_dist = 1000  # arbitrarily high value
  closest_wall = None
  for (m, wall) in walls_hit:
    if m < m_dist and m > 0:
      m_dist = m
      closest_wall = wall

  # calculate angle of incidence to wall
  ax = closest_wall[0]
  ay = closest_wall[1]
  bx = closest_wall[2]
  by = closest_wall[3]
  numerator = math.cos(theta)*(ay - by) + math.sin(bx - ax)
  denominator = math.sqrt((ay - by)**2 + (bx - ax)**2)
  beta = math.acos(numerator / denominator)
  if math.degrees(beta) > 60:
    # ignore

  K = 0.1
  std_dev = 2.5
  likelihood = math.exp((-((z - m)**2) / (2 * std_dev)) + K)

  return (likelihood, beta)

def normalise():
  weight_sum = sum(self.weights)
  for i in range(NUM_PARTICLES):
    self.weights[i] = self.weights[i] / weight_sum

def resample():
  cum_weights = [0] * NUM_PARTICLES
  cum_weight = 0
  for i in range(NUM_PARTICLES):
    cum_weight += self.weights[i]
    cum_weights[i] = cum_weight
  new_particles = [0] * NUM_PARTICLES
  for i in range(NUM_PARTICLES):
    rand = random.random()
    index = self.get_intersection(rand, cum_weights)
    new_particles[i] = self.particles[index]
    self.weights[i] = 1 / NUM_PARTICLES
  self.particels = new_particles

def get_intersection(rand, cum_weights):
  for i in range(NUM_PARTICLES):
    if value < cum_weights[i]:
      return i


# putting it all together
curr_position_estimate = [(84, 30, 0)]
for (x, y) in waypoints_to_navigate:
  mover.navigate_to_waypoint(x, y)
  canvas.drawParticles(mover.particles)
  got_sensor = False
  while not got_sensor:
    try:
      z = BP.get_sensor(BP.PORT_1)
      got_sensor = True
    except brickpi3.SensorError as error:
      print(error)
  num_large_beta_values = 0
  for i in range(NUM_PARTICLES):
    particle = mover.particles[i]
    (likelihood, beta) = mover.calculate_likelihood(particle[0], particle[1],
                                                    particle[2], z)
    if beta > 60:
      num_large_beta_values += 1  # ignore the calculated likelihood
    else:
      mover.weights[i] = likelihood
  mover.normalise()
  if num_large_beta_values < 50:  # only resample if there are a few garbage values
    mover.resample()

  canvas.drawParticles(mover.particles)
  mover.update_curr_position_estimate()

```


# signature mapping
```
def characterise_location():
  signature = []
  for angle in range(360):
    distance = BP.get_sensor(BP.PORT_1)
    signature.append(distance)
    rotate_sensor(1)
  return signature

def compare_signatures(sg1, sg2):
  diff_squared = []
  for i in range(360):
    dist = 0
    for j in range(360):
      dist += (sg1[i] - sg2[j])**2
    diff_squared.append(i, j, dist)
  diff_squared.sort()  # sort by dist
  return diff_squared[0]

def recognise_location():
  signature = characterise_location()
  least_dist = 10000
  angle_offset = None
  location = None
  for sig in learned_locations:
    (i, j, diff) = compare_signatures(signature, sig)
    if diff < least_dist:
      least_dist = diff
      angle_offset = i - j
      location = sig
    if least_dist > K:
      # the robot is likely to not be in any of the saved locations
  return (location, angle_offset)

```



# occupancy mapping
