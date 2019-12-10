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

# note that variance of e, f, g should scale with rotation and distance

def navigate_to_waypoint(x, y):
  x_diff = x - curr_position_estimate[0]
  y_diff = y - curr_position_estimate[1]
  distance = math.sqrt(x_diff**2, y_diff**2)
  angle_diff = math.degrees(math.atan2(y_diff, x_diff))
  angle = angle_diff - curr_position_estimate[3]

  # scale it to be -180 < angle < 180, so that robot always rotates the lesser angle
  if angle < -180:
    angle += 360
  if angle > 180:
    angle -= 360

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
    if denominator == 0:
      continue  # skip this wall
    m = numerator / denominator

    # check if the sonar will hit within limits of the wall
    x_pos = x + m * math.cos(theta)
    y_pos = y + m * math.sin(theta)

    if not point_lies_on_wall((x, y), (ax, ay), (bx, by)):
      continue  # continue to next wall

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

  K = 0.1
  std_dev = 2.5
  likelihood = math.exp((-((z - m)**2) / (2 * std_dev**2)) + K)

  return (likelihood, beta)

# bounding box to allow for floating point errors
def point_lies_on_wall(p, a, b):
  ab = distance(a, b)
  ap = distance(a, p)
  bp = distance(b, p)
  return math.abs(ab - ap - bp) < 0.2

def distance(x, y):
  return math.sqrt((y[0] - x[0])**2 + (y[1] - x[1])**2)

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
      mover.weights[i] = likelihood * mover.weights[i]
  mover.normalise()
  if num_large_beta_values < 50:  # only resample if there are a few garbage values
    mover.resample()

  canvas.drawParticles(mover.particles)
  mover.update_curr_position_estimate()


def update_curr_position_estimate():
  mean_x = 0
  mean_y = 0
  mean_theta = 0
  for particle in particles:
    mean_x += particle.x * particle.weight
    mean_y += particle.y * particle.weight
    if particle.angle > 180:
      particle.angle -= 360
    mean_angle += particle.angle * particle.weight
  if mean_theta < 0:
    mean_theta += 360
  curr_position_estimate[0] = mean_x
  curr_position_estimate[1] = mean_y
  curr_position_estimate[2] = mean_theta

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

```
# initialise grid with all zeroes

# make sonar measurement

# find grid boxes which lie within the angle range (15-20deg)

# for boxes with dist < z - sigma -- subtract 2

# for boxes with dist within z +- signma -- add 2

def update_occupancy_map(x, y, theta, z, alpha):
  for i in range(500):
    for j in range(500):
      x_diff = i - x
      y_diff = j - y
      distance = math.sqrt(y_diff**2 + x_diff**2)
      angle_offset = math.atan2(y_diff, x_diff)
      angle = angle_offset - theta
      if angle < -180:
        angle += 360
      if angle > 180:
        angle -= 360
      # take note of angular wrap around (1, 359 is within 5degrees)
      if math.abs(angle - alpha) >= 5:
        continue
      if distance < z - 4:
        occupancyMap[i][j] -= 2
	  if math.abs(distance - z) <= 4:
		occupancyMap[j][j] += 5
```

# clip (for threshold values of wheel velocity)
```
def clip(n, lo, hi):
  return max(lo, min(n, hi))
```

# radius, angle of differential drive
R = (W * (vR + vL)) / (2 * (vR - vL))

delta_theta = ((vR - vL) * delta_t) / W


# endpoints of intersection with wall
```
x_coord = m * math.cos(theta)
Y_coord = m * math.sin(theta)
```


# angle of incidence to wall
ignore reading if beta > 60
```
numerator = math.cos(theta)*(ay - by) + math.sin(theta)*(bx - ax)
denominator = math.sqrt((ay - by)**2 + (bx - ax)**2)
beta = math.acos(numerator / denominator)
```

# likelihood function
```
K = 0.01
sigma = 2
math.exp((-(z - m)**2) / 2 * sigma**2) + K
```

# m calculation
```
numerator = (by - ay)*(ax - x) - (bx - ax)*(ay - y)
denominator =  (by - ay)*math.cos(theta) - (bx - ax)*math.sin(theta)
m = numerator / denominator
```

# circular motion
```
x + R * (math.sin(delta_theta + theta) - math.sin(theta))
y - R * (math.cos(delta_theta + theta) - math.cos(theta))
theta + delta_theta
```

# median filtering
sensor sometimes gives rubbish values --> get 5 measurements and take median
however this makes the system less responsive

# cascade control
- output of one control loop is used as the input for the other
- abstraction hides details of motor/encoder control from sensor control
- top level controller may request velocity changes too quickly --> exceed bandwidth of lower level controller

# particle filtering
- cloud of weighted particles represent distribution of robot position
- simple to implement
- able to represent more than one peak
- however, hard to represent detailed shape when there are few particles
- high number of particles is computationally expensive and hard to scale
- if initial position is unknown, may need to take multiple movements and measurements to find its right location


# PID control
u(t) = kp*e(t) + ki*integrate(e(T)dT) + kd*(de(t)/dt)

proportional term:
- e(t) is the difference between expected and actual (ie. the error)
- high kp results in large change in output power for a given change in error
- if kp is too high, the system can become unstable and keep overshooting
- if kp is too low, there will be a small output response to a large input error, resulting in a less responsive controller

integral term:
- sum of accumulated previous errors
- this aims to reduce residual steady-state error that occurs from a pure proportional controller
- however since it accumulates errors, a high ki could cause the present value to overshoot the setpoint value

differential term:
- power output is proportional to the rate of change in error
- a high kd reduces settling time and improves stability of the system

```
previous_error = 0
integral = 0
while True:
  error = expected - actual
  integral += error * time
  derivative = (error - previous_error) / time
  total_error = kp * error + ki * integral + kd * derivative
  previous_error = error
  time.sleep(time)
```


# SLAM
Used when the robot is in a location it has never been in  before
- build a map incrementally
- localise with respect to that map as it grows and is gradually refined

Features for SLAM
- estimate 3D position of object in the room (x,y,z)
- distinctive form each other, easily recognisable from different viewpoints

Propagating uncertainty
- jointly estimate both probabilities at the same time (environment and robot position)
- assume that the world is static; only the robot is moving
- extend to higher dimensions (extend the state vector)
- don't really use particle filter for SLAM (exponential amount of calculations)

Extended Kalman filter
- when the robot initialises B and C, they inherit the robot's uncertainty + a little bit more
- when the robot re-measures A, it closes a loop --> decreases uncertainty of where the robot is + B and C


#### SLAM Algorithms
- Active Vision
  - measures uncertainty of landmarks + track previous locations
- Ring of Sensors
  - make sonar measurements in all directions
  - estimate landmarks in 2D
- Single Camera (MonoSLAM)
  - useful for AR, ARkit, Hololens
  - infer depth over multiple views (via triangulation)
- Pure topological map
  - use place recognition and link them together to find geometric map
- add metric information to topological map
  - apply pose graph optimisation (relaxation) algorithm

Limitations of Metric SLAM
- poor computational scaling of probabilistic filters (need to update at high frequency)
- growth in uncertainty at large distances from map origin
- gaussian function may not be representative of the distribution (when particles rotate --> banana like shape)

local metric mapping to estimate smaller local map --> jigsaw to combine the maps together (using place recognition) --> global optimisation when you find a loop
