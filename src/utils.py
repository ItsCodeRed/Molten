import math
from objects import Vector3

def find_acceleration(target, car, time, gravity = 650):
    #Finds the acceleration required for a car to reach a target in a specific amount of time
    d = target - car.location
    dvx = ((d[0]/time) - car.velocity[0]) / time
    dvy = ((d[1]/time) - car.velocity[1]) / time
    dvz = (((d[2]/time) - car.velocity[2]) / time) + (gravity * time)
    return Vector3(dvx,dvy,dvz)

def perdict_car_location(car, time, gravity = 650):
    #Finds the cars location after a certain amount of time
    return car.location + car.velocity * time + 0.5 * gravity * time ** 2

def eta(car, target, direction, distance):
    car_to_target = target - car.location

    forward_angle = abs(abs(direction.angle(car.forward)) - (math.pi / 2) if abs(direction.angle(car.forward)) > math.pi / 2 else 0)

    int_velocity = math.cos(car.velocity.angle(car_to_target)) * car.velocity.magnitude()

    distance = distance + find_turn_radius(car.velocity.magnitude()) * forward_angle

    boosting_acceleration = 991.666
    driving_acceleration = 1500 - int_velocity

    time_until_no_boost = car.boost / 33.3

    boosted_acceleration = driving_acceleration + boosting_acceleration
    final_boost_velocity = int_velocity + boosted_acceleration * time_until_no_boost

    distance_with_boost = int_velocity * time_until_no_boost + (1/2) * boosted_acceleration * math.pow(time_until_no_boost, 2)

    if final_boost_velocity > 2300:
        time_until_max = (2300 - int_velocity) / boosted_acceleration
        distance_covered = int_velocity * time_until_max + (1/2) * boosted_acceleration * math.pow(time_until_max, 2)
        if distance_covered < distance:
            return time_until_max + (distance - distance_covered) / 2300, True
        else:
            final_velocity = math.sqrt(math.pow(int_velocity, 2) + 2 * boosted_acceleration * distance)
            return (final_velocity - int_velocity) / boosted_acceleration, True
    else:
        max_speed = cap(final_boost_velocity, 1410, 2300)

        if distance_with_boost > distance:
            final_velocity = math.sqrt(math.pow(int_velocity, 2) + 2 * boosted_acceleration * distance)
            return (final_velocity - int_velocity) / boosted_acceleration, True
        else:
            if final_boost_velocity < 1410:
                time_until_max = (1410 - final_boost_velocity) / driving_acceleration
                distance_covered = final_boost_velocity * time_until_max + (1/2) * driving_acceleration * math.pow(time_until_max, 2)
                if distance_covered + distance_with_boost < distance:
                    return time_until_no_boost + time_until_max + (distance - distance_with_boost - distance_covered) / 1410, True
                else:
                    final_velocity = math.sqrt(math.pow(final_boost_velocity, 2) + 2 * driving_acceleration * (distance - distance_with_boost))
                    return time_until_no_boost + (final_velocity - int_velocity) / driving_acceleration, True
            else:
                distance_remaining = distance - distance_with_boost
                return time_until_no_boost + (distance_remaining / max_speed), True

def cap(x, low, high):
    #caps/clamps a number between a low and high value
    if x < low:
        return low
    elif x > high:
        return high
    return x

def length_by_angle(angle):
    if angle < math.pi / 2:
        length = 83.355
        default_angle = math.atan(73/42)
        new_angle = default_angle + abs(angle)
        return math.sin(new_angle) * length, math.cos(new_angle) * length
    else:
        length = 65.299
        default_angle = math.atan(-50/42)
        new_angle = default_angle + abs(angle)
        return math.sin(new_angle) * length, math.cos(new_angle) * length

def default_orient(agent, local_target, direction = 1.0):
    #points the car towards a given local target.
    #Direction can be changed to allow the car to steer towards a target while driving backwards
    local_target *= direction
    up = agent.me.local(Vector3(0,0,1)) #where "up" is in local coordinates
    target_angles = [
        math.atan2(local_target[2],local_target[0]), #angle required to pitch towards target
        math.atan2(local_target[1],local_target[0]), #angle required to yaw towards target
        math.atan2(up[1],up[2])]                     #angle required to roll upright
    #Once we have the angles we need to rotate, we feed them into PD loops to determing the controller inputs
    agent.controller.steer = steer(target_angles[1], 0) * direction
    agent.controller.pitch = steer(target_angles[0], agent.me.angular_velocity[1]/6)
    agent.controller.yaw = steer(target_angles[1], -agent.me.angular_velocity[2]/6)
    agent.controller.roll = steer(target_angles[2], agent.me.angular_velocity[0]/3)
    #Returns the angles, which can be useful for other purposes
    return target_angles

def default_throttle(agent, target_speed, direction = 1.0):
    #accelerates the car to a desired speed using throttle and boost
    car_speed = agent.me.local(agent.me.velocity)[0]
    t = (target_speed * direction) - car_speed
    agent.controller.throttle = cap((t**2) * sign(t)/1000, -1.0, 1.0)
    agent.controller.boost = True if t > 150 and car_speed < 2275 and agent.controller.throttle == 1.0 else False
    return car_speed

def in_field(point,radius):
    #determines if a point is inside the standard soccer field
    point = Vector3(abs(point[0]),abs(point[1]),abs(point[2]))
    if point[0] > 4080 - radius:
        return False
    elif point[1] > 5900 - radius:
        return False
    elif point[0] > 880 - radius and point[1] > 5105 - radius:
        return False
    elif point[0] > 2650 and point[1] > -point[0] + 8025 - radius:
        return False
    return True    

def distance_to_wall(point):
    #determines how close the car is to the wall
    abs_point = Vector3(abs(point[0]), abs(point[1]), abs(point[2]))

    distance_to_side_wall = 4096 - abs_point[0]
    distance_to_back_wall = 5120 - abs_point[1]
    distance_to_corner = math.sqrt(2) * ((8064 - abs_point[0] - abs_point[1]) / 2)
    if distance_to_corner > distance_to_side_wall < distance_to_back_wall:
        return distance_to_side_wall, Vector3(sign(point[0]), 0, 0)
    elif distance_to_corner > distance_to_side_wall > distance_to_back_wall:
        return distance_to_back_wall, Vector3(0, sign(point[1]), 0)
    else:
        return distance_to_corner, Vector3(sign(point[0]) / math.sqrt(2), sign(point[1]) / math.sqrt(2), 0)

def is_on_wall(point, sloped):
    distance = distance_to_wall(point)[0]

    if sloped:
        if (4 * distance) / (point[2] + 200) < 1:
            return True
        else:
            return False
    else:
        if distance < 150 and point[2] > 300:
            return True
        else:
            return False

def find_slope(shot_vector,car_to_target):
    #Finds the slope of your car's position relative to the shot vector (shot vector is y axis)
    #10 = you are on the axis and the ball is between you and the direction to shoot in
    #-10 = you are on the wrong side
    #1.0 = you're about 45 degrees offcenter
    d = shot_vector.dot(car_to_target)
    e = abs(shot_vector.cross((0,0,1)).dot(car_to_target))
    return cap(d / e if e != 0 else 10*sign(d), -3.0,3.0)

def post_correction(ball_location, left_target, right_target):
    #this function returns target locations that are corrected to account for the ball's radius
    #If the left and right post swap sides, a goal cannot be scored
    ball_radius = 120 #We purposly make this a bit larger so that our shots have a higher chance of success
    goal_line_perp = (right_target - left_target).cross((0,0,1))
    left = left_target + ((left_target - ball_location).normalize().cross((0,0,-1))*ball_radius)
    right = right_target + ((right_target - ball_location).normalize().cross((0,0,1))*ball_radius)
    left = left_target if (left-left_target).dot(goal_line_perp) > 0.0 else left
    right = right_target if (right-right_target).dot(goal_line_perp) > 0.0 else right
    swapped = True if (left - ball_location).normalize().cross((0,0,1)).dot((right - ball_location).normalize()) > -0.1 else False
    return left,right,swapped

def quadratic(a,b,c):
    #Returns the two roots of a quadratic
    inside = math.sqrt((b*b) - (4*a*c))
    if a != 0:
        return (-b + inside)/(2*a),(-b - inside)/(2*a)
    else:
        return -1,-1

def shot_valid(agent, shot, threshold = 40):
    #Returns True if the ball is still where the shot anticipates it to be
    #First finds the two closest slices in the ball prediction to shot's intercept_time
    #threshold controls the tolerance we allow the ball to be off by
    slices = agent.get_ball_prediction_struct().slices
    soonest = 0
    latest = len(slices)-1
    while len(slices[soonest:latest+1]) > 2:
        midpoint = (soonest+latest) // 2
        if slices[midpoint].game_seconds > shot.intercept_time:
            latest = midpoint
        else:
            soonest = midpoint
    #preparing to interpolate between the selected slices
    dt = slices[latest].game_seconds - slices[soonest].game_seconds
    time_from_soonest = shot.intercept_time - slices[soonest].game_seconds
    slopes = (Vector3(slices[latest].physics.location) - Vector3(slices[soonest].physics.location)) * (1/dt)
    #Determining exactly where the ball will be at the given shot's intercept_time
    predicted_ball_location = Vector3(slices[soonest].physics.location) + (slopes * time_from_soonest)
    #Comparing predicted location with where the shot expects the ball to be
    return (shot.ball_location - predicted_ball_location).magnitude() < threshold

def side(x):
    #returns -1 for blue team and 1 for orange team
    if x == 0:
        return -1
    return 1
    
def sign(x):
    #returns the sign of a number, -1, 0, +1
    if x < 0.0:
        return -1
    elif x > 0.0:
        return 1
    else:
        return 0.0
    
def steer(angle, rate):
    #A Proportional-Derivative control loop used for defaultPD
    return cap(((35*(angle+rate))**3)/10, -1.0, 1.0)

def find_turn_radius(speed):
    if (speed <= 500):
        radius = lerp(145, 251, speed / 500)
    elif (speed <= 1000):
        radius = lerp(251, 425, (speed - 500) / 500)
    elif (speed <= 1500):
        radius = lerp(425, 727, (speed - 1000) / 500)
    elif (speed <= 1750):
        radius = lerp(727, 909, (speed - 1500) / 250)
    else:
        radius = lerp(909, 1136, (speed - 1750) / 550)

    return radius

def lerp(a, b, t):
    #Linearly interpolate from a to b using t
    #For instance, when t == 0, a is returned, and when t == 1, b is returned
    #Works for both numbers and Vector3s
    return (b - a) * t + a

def invlerp(a, b, v):
    #Inverse linear interpolation from a to b with value v
    #For instance, it returns 0 if v == a, and returns 1 if v == b, and returns 0.5 if v is exactly between a and b
    #Works for both numbers and Vector3s
    return (v - a)/(b - a)

def pom_sqrt(x):
    if x < 0:
        return -math.sqrt(abs(x))
    else:
        return math.sqrt(x)
