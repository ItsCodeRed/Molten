import math
from util.objects import Vector3

def find_acceleration(target, car, time, gravity = 650):
    #Finds the acceleration required for a car to reach a target in a specific amount of time
    d = target - car.location
    dvx = ((d[0]/time) - car.velocity[0]) / time
    dvy = ((d[1]/time) - car.velocity[1]) / time
    dvz = (((d[2]/time) - car.velocity[2]) / time) + (gravity * time)
    return Vector3(dvx,dvy,dvz)

def perdict_car_location(car, time, gravity = 650):
    #Finds the cars location after a certain amount of time
    return car.location + car.velocity * time + 0.5 * Vector3(0, 0, -gravity) * time ** 2

def eta(car, target, direction, distance):
    car_to_target = target - car.location

    forward_angle = abs(abs(direction.angle(car.forward)) - (math.pi / 2) if abs(direction.angle(car.forward)) > math.pi / 2 else 0)

    int_velocity = math.cos(car.velocity.angle(car_to_target)) * car.velocity.magnitude()

    # distance = distance + find_turn_radius(car.velocity.magnitude()) * forward_angle

    boosting_acceleration = 991.666
    driving_acceleration = 1500

    time_until_no_boost = car.boost / 33.3

    boosted_acceleration = driving_acceleration + boosting_acceleration
    final_boost_velocity = int_velocity + boosted_acceleration * time_until_no_boost

    distance_with_boost = int_velocity * time_until_no_boost + (1/2) * boosted_acceleration * math.pow(time_until_no_boost, 2)

    if final_boost_velocity > 2300:
        time_until_max = (2300 - int_velocity) / boosted_acceleration
        distance_covered = int_velocity * time_until_max + (1/2) * boosted_acceleration * math.pow(time_until_max, 2)
        if distance_covered < distance:
            return time_until_max + (distance - distance_covered) / 2300
        else:
            final_velocity = math.sqrt(math.pow(int_velocity, 2) + 2 * boosted_acceleration * distance)
            return (final_velocity - int_velocity) / boosted_acceleration
    else:
        max_speed = cap(final_boost_velocity, 1410, 2300)

        if distance_with_boost > distance:
            final_velocity = math.sqrt(math.pow(int_velocity, 2) + 2 * boosted_acceleration * distance)
            return (final_velocity - int_velocity) / boosted_acceleration
        else:
            if final_boost_velocity < 1410:
                time_until_max = (1410 - final_boost_velocity) / driving_acceleration
                distance_covered = final_boost_velocity * time_until_max + (1/2) * driving_acceleration * math.pow(time_until_max, 2)
                if distance_covered + distance_with_boost < distance:
                    return time_until_no_boost + time_until_max + (distance - distance_with_boost - distance_covered) / 1410
                else:
                    final_velocity = math.sqrt(math.pow(final_boost_velocity, 2) + 2 * driving_acceleration * (distance - distance_with_boost))
                    return time_until_no_boost + (final_velocity - int_velocity) / driving_acceleration
            else:
                distance_remaining = distance - distance_with_boost
                return time_until_no_boost + (distance_remaining / max_speed)

def cap(x, low, high):
    #caps/clamps a number between a low and high value
    if x < low:
        return low
    elif x > high:
        return high
    return x

def car_ball_collision_offset(angle):
    ball_length = 94.41
    # fc = front corner
    fc_length = 84.083
    fc_int_angle = math.atan(-41.9/72.9)
    fc_new_angle = fc_int_angle + angle
    # fl = front line
    fl_int_angle = math.pi / 2
    fl_new_angle = fl_int_angle + angle
    # fp = front perpindicular
    fp_new_angle = angle

    # sl = side line
    sl_int_angle = -math.pi
    sl_new_angle = sl_int_angle + angle
    # sp = side perpindicular
    sp_int_angle = -math.pi / 2
    sp_new_angle = sp_int_angle + angle
    
    # bc = back corner
    bc_length = 65.235
    bc_int_angle = math.atan(41.9/50) + math.pi
    bc_new_angle = bc_int_angle + angle
    # bl = back line
    bl_int_angle = -math.pi / 2
    bl_new_angle = bl_int_angle + angle
    # bp = back perpindicular
    bp_int_angle = -math.pi
    bp_new_angle = bp_int_angle + angle

    if angle < math.pi / 2:
        collision_point_y = -math.sin(fp_new_angle) * ball_length
        front_corner = Vector3(math.cos(fc_new_angle) * fc_length, math.sin(fc_new_angle) * fc_length, 0)
        if collision_point_y > front_corner.y:
            fl_vector = Vector3(math.cos(fl_new_angle), math.sin(fl_new_angle), 0)
            multiplier = (collision_point_y - front_corner.y) / fl_vector.y
            collision_point = front_corner + fl_vector * multiplier
            return collision_point.x + math.cos(fp_new_angle) * ball_length
        else:
            collision_point_y = -math.sin(sp_new_angle) * ball_length
            front_corner = Vector3(math.cos(fc_new_angle) * fc_length, math.sin(fc_new_angle) * fc_length, 0)
            if collision_point_y < front_corner.y:
                sl_vector = Vector3(math.cos(sl_new_angle), math.sin(sl_new_angle), 0)
                multiplier = (collision_point_y - front_corner.y) / sl_vector.y
                collision_point = front_corner + sl_vector * multiplier
                return collision_point.x + math.cos(sp_new_angle) * ball_length
            else:
                return front_corner.x + math.cos(math.asin(front_corner.y/ball_length)) * ball_length
    else:
        collision_point_y = -math.sin(bp_new_angle) * ball_length
        back_corner = Vector3(math.cos(bc_new_angle) * bc_length, math.sin(bc_new_angle) * bc_length, 0)
        if collision_point_y < back_corner.y:
            bl_vector = Vector3(math.cos(bl_new_angle), math.sin(bl_new_angle), 0)
            multiplier = (collision_point_y - back_corner.y) / bl_vector.y
            collision_point = back_corner + bl_vector * multiplier
            return collision_point.x + math.cos(bp_new_angle) * ball_length
        else:
            collision_point_y = -math.sin(sp_new_angle) * ball_length
            back_corner = Vector3(math.cos(bc_new_angle) * bc_length, math.sin(bc_new_angle) * bc_length, 0)
            if collision_point_y > back_corner.y:
                sl_vector = Vector3(math.cos(angle), math.sin(angle), 0)
                multiplier =(collision_point_y - back_corner.y) / sl_vector.y
                collision_point = back_corner + sl_vector * multiplier
                return collision_point.x + math.cos(sp_new_angle) * ball_length
            else:
                return back_corner.x + math.cos(math.asin(back_corner.y/ball_length)) * ball_length

def find_jump_time(height, double_jump=False):
    int_jump_velocity = 291.667
    gravity_acceleration = -650
    jump_acceleration = 1458.333374
    jump_stop_time = 0.2

    height_after_jump = int_jump_velocity * jump_stop_time + (1 / 2) * (gravity_acceleration + jump_acceleration) * math.pow(jump_stop_time, 2)
    double_jump_multiplier = 2 if double_jump else 1

    if height_after_jump < height:
        int_velocity_after_jump = double_jump_multiplier * int_jump_velocity + (gravity_acceleration + jump_acceleration) * jump_stop_time
        fin_velocity_after_jump = pom_sqrt(math.pow(int_velocity_after_jump, 2) + 2 * gravity_acceleration * (height - height_after_jump))
        return jump_stop_time + (fin_velocity_after_jump - int_velocity_after_jump) / gravity_acceleration
    else:
        fin_jump_velocity = math.sqrt(math.pow(int_jump_velocity, 2) + 2 * (gravity_acceleration + jump_acceleration) * height)
        return (fin_jump_velocity - int_jump_velocity) / (gravity_acceleration + jump_acceleration)

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

def roll_orient(agent, local_target, direction = 1.0):
    #points the car towards a given local target.
    #Direction can be changed to allow the car to steer towards a target while driving backwards
    local_target *= direction
    yaw_angle = math.atan2(local_target[1],local_target[0])
    target_angles = [
        math.atan2(local_target[2],local_target[0]) * abs(math.cos(yaw_angle)), #angle required to pitch towards target
        0, #no yaw
        math.atan2(local_target[2],abs(local_target[1])) * -sign(local_target[1]) * abs(math.sin(yaw_angle))] #angle required to roll towards target
    #Once we have the angles we need to rotate, we feed them into PD loops to determing the controller inputs
    agent.controller.pitch = steer(target_angles[0], agent.me.angular_velocity[1]/6)
    agent.controller.roll = steer(target_angles[2], agent.me.angular_velocity[0]/3)
    #Returns the angles, which can be useful for other purposes
    return target_angles

def freestyle_orient(agent, local_target, direction = 1.0):
    #points the car towards a given local target.
    #Direction can be changed to allow the car to steer towards a target while driving backwards
    local_target *= direction
    target_angles = [
        math.atan2(local_target[2],local_target[0]), #angle required to pitch towards target
        math.atan2(local_target[1],local_target[0]), #angle required to yaw towards target
        math.atan2(local_target[1],local_target[2])]
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
    speed_difference = (target_speed * direction) - car_speed
    agent.controller.throttle = cap((speed_difference**2) * sign(speed_difference)/500, -1.0, 1.0)
    agent.controller.boost = True if speed_difference > 150 and car_speed < 2275 and agent.controller.throttle == 1.0 else False
    return car_speed

def default_drive(agent, local_target, target_speed):
    default_throttle(agent, target_speed, 1)
    angles = default_orient(agent, local_target, 1)

    agent.controller.boost = False if abs(angles[1]) > 0.3 or agent.me.airborne else agent.controller.boost
    agent.controller.handbrake = True if abs(angles[1]) > 2 or local_target.magnitude() < find_turn_radius(abs(agent.me.local(agent.me.velocity)[0])) * abs(angles[1]) else agent.controller.handbrake

    return angles
    
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

def is_on_wall(point, try_to_reach):
    distance = distance_to_wall(point)[0]

    if try_to_reach:
        return distance < 300 < point[2]
    else:
        return distance < 150

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

def pom_sqrt(x, both=False):
    if x < 0 :
        return -math.sqrt(abs(x))
    else:
        return math.sqrt(x)

def find_shot_angle(s, x, y):
    g = 650 # gravity constant
    
    if s**4 - g * (g * x**2 + 2 * y * s**2) < 0:
        return math.pi / 4 # not possible to hit target at speed without hitting ground, so angle defaults to 0
    else:
        shot_angle_1 = math.atan2(s**2 + math.sqrt(s**4 - g * (g * x**2 + 2 * y * s**2)), g * x)
        shot_angle_2 = math.atan2(s**2 - math.sqrt(s**4 - g * (g * x**2 + 2 * y * s**2)), g * x)
        straight_angle = math.atan2(y,x)

        return shot_angle_1 if abs(straight_angle - shot_angle_1) < abs(straight_angle - shot_angle_2) else shot_angle_2 # returns the smallest angle to hit target

def get_max_angle(time, acceleration, int_ang_vel):
    time_until_max = (5.5 - int_ang_vel) / acceleration
    
    if time < time_until_max:
        fin_ang_vel = int_ang_vel + acceleration * time
        return ((int_ang_vel + fin_ang_vel) / 2) * time
    else:
        return ((int_ang_vel + 5.5) / 2) * time_until_max + 5.5 * (time - time_until_max)

def length_by_angle(angle):
    if angle < math.pi / 2:
        length = 84.3
        default_angle = math.atan(73/42)
        new_angle = default_angle + abs(angle)
        return math.sin(new_angle) * length, math.cos(new_angle) * length
    else:
        length = 65.299
        default_angle = math.atan(-50/42)
        new_angle = default_angle + abs(angle)
        return math.sin(new_angle) * length, math.cos(new_angle) * length
