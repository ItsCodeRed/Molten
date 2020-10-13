from utils import *

gravity: Vector3 = Vector3(0, 0, -650)

# Aerial constants
max_speed: float = 2300
boost_accel: float = 1060
throttle_accel: float = 200 / 3
boost_per_second: float = 30

# Jump constants

jump_speed: float = 291.667
jump_acc = 1458.3333
jump_min_duration = 0.025
jump_max_duration = 0.2

class atba():
    #An example routine that just drives towards the ball at max speed
    def run(self, agent):
        relative_target = agent.ball.location - agent.me.location
        local_target = agent.me.local(relative_target)
        default_orient(agent, local_target)
        default_throttle(agent, 1600)
        agent.pop()

class aerial():
    def __init__(self, ball_location, intercept_time, shot_vector, direction):
        self.ball_location = ball_location
        self.shot_vector = shot_vector
        self.target_location = self.ball_location - self.shot_vector * 170
        self.intercept_time = intercept_time
        self.direction = direction
        self.jump_threshold = 500
        self.launching = False
        self.jumping = True
        self.time = -1
        self.jump_time = -1
        self.counter = 0
    def update(self, ball_location, intercept_time, shot_vector, direction):
        self.ball_location = ball_location
        self.shot_vector = shot_vector
        self.target_location = self.ball_location - self.shot_vector * 170
        self.intercept_time = intercept_time
        self.direction = direction
    def run(self, agent):

        self.jumping = not agent.me.airborne

        if not self.launching and self.jumping:
            raw_time_remaining = self.intercept_time - agent.time
            time_remaining = cap(raw_time_remaining, 0.001, 10.0)

            car_to_target = self.target_location - agent.me.location

            distance = car_to_target.magnitude()
            speed_required = distance / time_remaining

            acceleration_required = find_acceleration(self.target_location, agent.me, time_remaining, 0 if not self.launching else 650)
            local_acceleration_required = agent.me.local(acceleration_required)

            horizontal_angle = Vector3(1, 0, 0).angle(agent.me.local(self.target_location))
            horizontal_offset = horizontal_angle * 500 if horizontal_angle < math.pi / 2 else (math.pi - horizontal_angle) * 500
            self.jump_threshold = cap(horizontal_offset, 200 if self.target_location.z > 400 else 300, 500)

            line_up_for_shot(self.ball_location, self.intercept_time, self.shot_vector, self.direction, 1).run(agent)

            if time_remaining < 0.01 or not shot_valid(agent, self, threshold=150):
                agent.pop()
                if agent.me.airborne:
                    agent.push(recovery())
            elif local_acceleration_required[2] > self.jump_threshold and local_acceleration_required[2] > local_acceleration_required.flatten().magnitude():
                self.launching = True 
        else:
            if self.time == -1:
                elapsed = 0
                self.time = agent.time
            else:
                elapsed = agent.time - self.time
            T = self.intercept_time - agent.time
            xf = agent.me.location + agent.me.velocity * T + 0.5 * gravity * T ** 2
            vf = agent.me.velocity + gravity * T
            if self.jumping:
                if self.jump_time == -1:
                    jump_elapsed = 0
                    self.jump_time = agent.time
                else:
                    jump_elapsed = agent.time - self.jump_time
                tau = jump_max_duration - jump_elapsed
                if jump_elapsed == 0:
                    vf += agent.me.orientation.up * jump_speed
                    xf += agent.me.orientation.up * jump_speed * T

                vf += agent.me.orientation.up * jump_acc * tau
                xf += agent.me.orientation.up * jump_acc * tau * (T - 0.5 * tau)

                vf += agent.me.orientation.up * jump_speed
                xf += agent.me.orientation.up * jump_speed * (T - tau)

                if jump_elapsed < jump_max_duration:
                    agent.controller.jump = True
                elif elapsed >= jump_max_duration and self.counter < 3:
                    agent.controller.jump = False
                    self.counter += 1
                elif elapsed < 0.3:
                    agent.controller.jump = True
                else:
                    self.jumping = jump_elapsed <= 0.3
            else:
                agent.controller.jump = 0

            delta_x = self.ball_location - xf
            direction = delta_x.normalize()
            if delta_x.magnitude() > 50:
                default_orient(agent, agent.me.local(delta_x))
            else:
                if self.shot_vector is not None:
                    default_orient(agent, agent.me.local(self.shot_vector))
                else:
                    default_orient(agent, agent.me.local(self.ball_location - agent.me.location))

            if elapsed < 0.3 and self.counter == 3:
                agent.controller.roll = 0
                agent.controller.pitch = 0
                agent.controller.yaw = 0
                agent.controller.steer = 0

            if agent.me.forward.angle3D(direction) < 0.3:
                if delta_x.magnitude() > 50:
                    agent.controller.boost = 1
                    agent.controller.throttle = 0
                else:
                    agent.controller.boost = 0
                    agent.controller.throttle = cap(0.5 * throttle_accel * T ** 2, 0, 1)
            else:
                agent.controller.boost = 0
                agent.controller.throttle = 0

            if T <= 0 or not shot_valid(agent, self, threshold=150):
                agent.pop()
                agent.push(recovery())

    def is_viable(self, agent, time: float):
        T = self.intercept_time - time
        xf = agent.me.location + agent.me.velocity * T + 0.5 * gravity * T ** 2
        vf = agent.me.velocity + gravity * T
        if not agent.me.airborne:
            vf += agent.me.orientation.up * (2 * jump_speed + jump_acc * jump_max_duration)
            xf += agent.me.orientation.up * (jump_speed * (2 * T - jump_max_duration) + jump_acc * (
                    T * jump_max_duration - 0.5 * jump_max_duration ** 2))

        delta_x = self.ball_location - xf
        f = delta_x.normalize()
        phi = f.angle3D(agent.me.forward)
        turn_time = 0.7 * (2 * math.sqrt(phi / 9))

        tau1 = turn_time * cap(1 - 0.3 / phi, 0, 1)
        required_acc = (2 * delta_x.magnitude()) / ((T - tau1) ** 2)
        ratio = required_acc / boost_accel
        tau2 = T - (T - tau1) * math.sqrt(1 - cap(ratio, 0, 1))
        velocity_estimate = vf + boost_accel * (tau2 - tau1) * f
        boos_estimate = (tau2 - tau1) * 30
        enough_boost = boos_estimate < 0.95 * agent.me.boost
        enough_time = abs(ratio) < 0.9

        in_goal = abs(agent.me.location.y) > 5200

        ball_over_goal = self.ball_location.z > 800 and abs(self.ball_location.x) < 900 and abs(self.ball_location.y) > 4800

        return velocity_estimate.magnitude() < 0.9 * max_speed and enough_boost and enough_time and (not in_goal or not ball_over_goal)


class line_up_for_shot():
    def __init__(self, ball_location, intercept_time, direction_vector, time_before_jump, accuracy):
        self.ball_location = ball_location
        self.direction_vector = direction_vector
        self.target_location = self.ball_location - self.direction_vector * 170
        self.intercept_time = intercept_time
        self.time_before_jump = time_before_jump
        self.accuracy_multiplier = accuracy
    def run(self, agent):
        raw_time_remaining = self.intercept_time - agent.time
        time_remaining = cap(raw_time_remaining, 0.001, 10.0)

        angle_from_ball = agent.me.forward.angle(self.direction_vector)

        car_to_ball = self.ball_location - agent.me.location

        side_of_shot = sign(self.direction_vector.cross((0, 0, 1)).dot(car_to_ball))
        shot_vector_perp = self.direction_vector.cross((0, 0, side_of_shot))
        horizontal_dodge_offset = shot_vector_perp * cap(math.sin(angle_from_ball) * 10, -1, 1) * abs(length_by_angle(angle_from_ball)[1])
        self.target_location = self.ball_location - self.direction_vector * (75 + length_by_angle(angle_from_ball)[0]) + horizontal_dodge_offset

        car_to_target = self.target_location - agent.me.location

        angle_from_shot = car_to_target.angle(self.direction_vector)

        car_to_ball = self.ball_location - agent.me.location

        towards_wall = distance_to_wall(self.ball_location)[1]

        if is_on_wall(self.target_location, True):
            if is_on_wall(agent.me.location, False):
                side_of_shot = sign(self.direction_vector.cross(towards_wall).dot(car_to_ball))
                car_to_target_perp = car_to_target.cross(towards_wall * side_of_shot)

                distance = car_to_target.magnitude()
                speed_required = distance / time_remaining
            else:
                side_of_shot = sign(self.direction_vector.cross((0, 0, 1)).dot(car_to_ball))
                car_to_target_perp = car_to_target.cross((0, 0, side_of_shot))

                self.target_location = self.target_location.flatten() + towards_wall * self.target_location[2] + Vector3(0, 0, 300)
                distance = (self.target_location - agent.me.location).magnitude()
                speed_required = distance / time_remaining
        else:
            side_of_shot = sign(self.direction_vector.cross((0, 0, 1)).dot(car_to_ball))
            car_to_target_perp = car_to_target.cross((0, 0, side_of_shot))

            distance = car_to_target.flatten().magnitude()
            speed_required = distance / time_remaining

        acceleration_required = find_acceleration(self.target_location, agent.me, time_remaining, 0 if not agent.me.airborne else 650)
        local_acceleration_required = agent.me.local(acceleration_required)
        
        adjustment = angle_from_shot * distance / (3 - self.accuracy_multiplier + angle_from_ball / math.pi)
        adjustment *= cap(time_remaining - self.time_before_jump * 2, 0, self.time_before_jump * 2) / (self.time_before_jump * 2 + 0.001)
        adjusted_target = self.target_location + ((car_to_target_perp.normalize() * adjustment) if not agent.me.airborne else 0)
        
        if abs(adjusted_target.x) > 4096 + distance_to_wall(agent.me.location)[0]:
            adjusted_target = adjusted_target + car_to_target_perp.normalize() * (((4000 + distance_to_wall(agent.me.location)[0]) * sign(adjusted_target.x) - adjusted_target.x) / car_to_target_perp.normalize().x)
        if abs(adjusted_target.y) > 5120 + distance_to_wall(agent.me.location)[0]:
            adjusted_target = adjusted_target + car_to_target_perp.normalize() * (((5000 + distance_to_wall(agent.me.location)[0]) * sign(adjusted_target.y) - adjusted_target.y) / car_to_target_perp.normalize().y)
        if abs(agent.me.location.y > 5120):
            adjusted_target.x = cap(adjusted_target.x, -750, 750)
            adjusted_target.y = cap(adjusted_target.y, -5120, 5120)

        if is_on_wall(self.target_location, True):
            if not is_on_wall(agent.me.location, False):
                adjusted_target = adjusted_target + towards_wall * self.target_location[2]

        boost_score = 9999
        best_boost_location = 0
        for boost in agent.pads + agent.boosts:
            distance_to_boost = (boost.location - agent.me.location).magnitude()
            angle_to_boost = (boost.location - agent.me.location).angle(adjusted_target - agent.me.location)
            if distance / (1 + agent.me.boost / 5) > angle_to_boost * distance_to_boost < boost_score and distance > 500 and angle_to_boost < 0.5 and boost.active:
                best_boost_location = boost.location
                boost_score = angle_to_boost * distance_to_boost

        if best_boost_location != 0:
            local_target = agent.me.local(best_boost_location - agent.me.location)
        else:
            local_target = agent.me.local(adjusted_target - agent.me.location)

        angles = default_orient(agent, local_target)
        default_throttle(agent, speed_required)

        agent.controller.boost = False if abs(angles[1]) > 0.3 or agent.me.airborne else agent.controller.boost
        agent.controller.handbrake = True if abs(angles[1]) > 2.3 else agent.controller.handbrake

        agent.line(agent.me.location, self.target_location)
        agent.line(self.target_location-Vector3(0, 0, 100), self.target_location + Vector3(0, 0, 100), [255, 0, 0])
        agent.line(adjusted_target-Vector3(0, 0, 100), adjusted_target+Vector3(0, 0, 100), [0, 255, 0])

class shoot():
    def __init__(self, ball_location, intercept_time, shot_vector, direction):
        self.ball_location = ball_location
        self.intercept_time = intercept_time
        self.shot_vector = shot_vector
        self.dodge_point = self.ball_location - (self.shot_vector * 170)
        self.direction = direction
        self.jump_threshold = 500
        self.jumping = False
        self.dodging = False
        self.counter = 0
        self.p = 0
        self.y = 0
    def update(self, ball_location, intercept_time, shot_vector, direction):
        self.ball_location = ball_location
        self.shot_vector = shot_vector
        self.dodge_point = self.ball_location - self.shot_vector * 170
        self.intercept_time = intercept_time
        self.direction = direction
    def run(self, agent):
        raw_time_remaining = self.intercept_time - agent.time
        time_remaining = cap(raw_time_remaining, 0.001, 10.0)

        angle_from_ball = agent.me.forward.angle(self.shot_vector)

        car_to_ball_location = self.ball_location - agent.me.location

        side_of_shot = sign(self.shot_vector.cross((0, 0, 1)).dot(car_to_ball_location))
        shot_vector_perp = self.shot_vector.cross((0, 0, side_of_shot))
        horizontal_dodge_offset = shot_vector_perp * cap(math.sin(angle_from_ball) * 10, -1, 1) * abs(length_by_angle(angle_from_ball)[1])
        self.dodge_point = self.ball_location - self.shot_vector * (75 + length_by_angle(angle_from_ball)[0]) + horizontal_dodge_offset

        dodge_length = Vector3(75 + length_by_angle(abs(angle_from_ball))[0], horizontal_dodge_offset.magnitude(), 0).magnitude()

        car_to_ball = self.dodge_point - agent.me.location
        distance = car_to_ball.flatten().magnitude()
        distance_to_ball = car_to_ball_location.flatten().magnitude()
        speed_required = distance / time_remaining

        speed_in_direction = (math.cos(agent.me.velocity.angle(car_to_ball)) * agent.me.velocity.magnitude()) if abs(agent.me.velocity.angle(car_to_ball)) < math.pi / 2 else 1

        miss_amount = math.sin(car_to_ball_location.angle(agent.me.velocity)) * distance_to_ball

        is_missing = miss_amount > dodge_length + 40

        can_dodge = distance_to_ball < dodge_length + 50 or distance < 50

        time_until_dodge = (math.cos(car_to_ball.angle(agent.me.velocity)) * distance) / speed_in_direction

        int_jump_velocity = 291.667
        gravity_acceleration = -650
        jump_acceleration = 1458.333374
        jump_stop_time = 0.2
        
        height_after_jump = int_jump_velocity * jump_stop_time + (1 / 2) * (gravity_acceleration + jump_acceleration) * math.pow(jump_stop_time, 2)

        if height_after_jump < cap(self.dodge_point[2] - agent.me.location[2], 0, 300):
            int_velocity_after_jump = int_jump_velocity + (gravity_acceleration + jump_acceleration) * jump_stop_time
            fin_velocity_after_jump = pom_sqrt(math.pow(int_velocity_after_jump, 2) + 2 * gravity_acceleration * (cap(self.dodge_point[2] - agent.me.location[2], 0, 300) - height_after_jump))
            time_to_jump = jump_stop_time + (fin_velocity_after_jump - int_velocity_after_jump) / gravity_acceleration
        else:
            fin_jump_velocity = math.sqrt(math.pow(int_jump_velocity, 2) + 2 * (gravity_acceleration + jump_acceleration) * cap(self.dodge_point[2] - agent.me.location[2], 0, 300))
            time_to_jump = (fin_jump_velocity - int_jump_velocity) / (gravity_acceleration + jump_acceleration)

        if not self.jumping:
            line_up_for_shot(self.ball_location, self.intercept_time, self.shot_vector, time_to_jump, 1).run(agent)

            if time_remaining < 0.01 or (speed_required - 2300) * time_remaining > 45 or not shot_valid(agent, self):
                agent.pop()
                if agent.me.airborne:
                    agent.push(recovery())
            elif time_to_jump > time_remaining and not is_missing:
                self.jumping = True 
        else:
            if (raw_time_remaining > 0.2 and not shot_valid(agent, self, 60)) or raw_time_remaining <= -1 or (not agent.me.airborne and self.counter > 0) or (agent.me.airborne and is_missing):
                agent.pop()
                agent.push(recovery())
            elif self.counter == 0 and raw_time_remaining > 0.05:
                #Initial jump to get airborne + we hold the jump button for extra power as required
                agent.controller.jump = True
            elif self.counter < 3:
                #make sure we aren't jumping for at least 3 frames
                agent.controller.jump = False
                self.counter += 1
            elif raw_time_remaining <= 0.1 and raw_time_remaining > -0.9 and can_dodge:
                #dodge in the direction of the shot_vector
                agent.controller.jump = True
                if not self.dodging:
                    vector = agent.me.local(self.shot_vector)
                    self.p = -vector[0]
                    self.y = vector[1]
                    self.dodging = True
                #simulating a deadzone so that the dodge is more natural
                agent.controller.pitch = self.p if abs(self.p) > 0.2 else 0 
                agent.controller.yaw = self.y if abs(self.y) > 0.3 else 0

class double_jump():
    def __init__(self, ball_location, intercept_time, shot_vector, direction):
        self.ball_location = ball_location
        self.intercept_time = intercept_time
        self.shot_vector = shot_vector
        self.dodge_point = self.ball_location - (self.shot_vector * 170)
        self.direction = direction
        self.jumping = False
        self.counter = 0
        self.time_of_jump = -1
    def update(self, ball_location, intercept_time, shot_vector, direction):
        self.ball_location = ball_location
        self.shot_vector = shot_vector
        self.dodge_point = self.ball_location - self.shot_vector * 170
        self.intercept_time = intercept_time
        self.direction = direction
    def run(self, agent):
        raw_time_remaining = self.intercept_time - agent.time
        time_remaining = cap(raw_time_remaining, 0.001, 10.0)

        angle_from_ball = agent.me.forward.angle(self.shot_vector)

        car_to_ball_location = self.ball_location - agent.me.location

        side_of_shot = sign(self.shot_vector.cross((0, 0, 1)).dot(car_to_ball_location))
        shot_vector_perp = self.shot_vector.cross((0, 0, side_of_shot))
        horizontal_dodge_offset = shot_vector_perp * cap(math.sin(angle_from_ball) * 10, -1, 1) * abs(length_by_angle(angle_from_ball)[1])
        self.dodge_point = self.ball_location - self.shot_vector * (75 + length_by_angle(angle_from_ball)[0]) + horizontal_dodge_offset

        dodge_length = 95 + length_by_angle(abs(angle_from_ball))[0]

        car_to_ball = self.dodge_point - agent.me.location
        distance = car_to_ball.flatten().magnitude()
        distance_to_ball = car_to_ball_location.magnitude()
        speed_required = distance / time_remaining

        int_jump_velocity = 291.667
        gravity_acceleration = -650
        jump_acceleration = 1458.333374
        jump_stop_time = 0.2
        
        height_after_jump = int_jump_velocity * jump_stop_time + (1 / 2) * (gravity_acceleration + jump_acceleration) * math.pow(jump_stop_time, 2)

        if height_after_jump < cap(self.dodge_point[2] - agent.me.location[2], 0, 300):
            int_velocity_after_jump = 2 * int_jump_velocity + (gravity_acceleration + jump_acceleration) * jump_stop_time
            fin_velocity_after_jump = pom_sqrt(math.pow(int_velocity_after_jump, 2) + 2 * gravity_acceleration * (cap(self.dodge_point[2] - agent.me.location[2], 0, 300) - height_after_jump))
            time_to_jump = jump_stop_time + (fin_velocity_after_jump - int_velocity_after_jump) / gravity_acceleration
        else:
            fin_jump_velocity = math.sqrt(math.pow(int_jump_velocity, 2) + 2 * (gravity_acceleration + jump_acceleration) * cap(self.dodge_point[2] - agent.me.location[2], 0, 300))
            time_to_jump = (fin_jump_velocity - int_jump_velocity) / (gravity_acceleration + jump_acceleration)

        perdicted_location = agent.me.location.flatten() + agent.me.velocity.flatten() * time_to_jump

        distance_off_mark = (self.dodge_point.flatten() - perdicted_location).magnitude()

        if not self.jumping:
            line_up_for_shot(self.ball_location, self.intercept_time, self.shot_vector, time_to_jump, 1).run(agent)

            if time_remaining < time_to_jump / 2 or (speed_required - 2300) * time_remaining > 45 or not shot_valid(agent, self):
                agent.pop()
                if agent.me.airborne:
                    agent.push(recovery())
            elif distance_off_mark < 30:
                self.jumping = True 
                if self.time_of_jump == -1:
                    elapsed = 0
                    self.time_of_jump = agent.time
        else:
            elapsed = agent.time - self.time_of_jump

            default_orient(agent, agent.me.local(self.shot_vector))

            if (raw_time_remaining > 0.2 and not shot_valid(agent, self, 60)) or raw_time_remaining <= -1 or (not agent.me.airborne and self.counter > 0):
                agent.pop()
                agent.push(recovery())
            elif self.counter == 0 and elapsed < 0.2:
                #Initial jump to get airborne + we hold the jump button for extra power as required
                agent.controller.jump = True
            elif self.counter < 3:
                #make sure we aren't jumping for at least 3 frames
                agent.controller.jump = False
                self.counter += 1
            elif self.counter < 6:
                agent.controller.jump = True
                agent.controller.pitch = 0
                agent.controller.yaw = 0
                agent.controller.roll = 0
                agent.controller.steer = 0
                self.counter += 1


class wall_hit():
    def __init__(self, ball_location, intercept_time, shot_vector, direction):
        self.ball_location = ball_location
        self.intercept_time = intercept_time
        self.shot_vector = shot_vector
        self.dodge_point = self.ball_location - (self.shot_vector * 170)
        self.direction = direction
        self.jumping = False
        self.dodging = False
        self.counter = 0
        self.p = 0
        self.y = 0
    def update(self, ball_location, intercept_time, shot_vector, direction):
        self.ball_location = ball_location
        self.shot_vector = shot_vector
        self.dodge_point = self.ball_location - self.shot_vector * 170
        self.intercept_time = intercept_time
        self.direction = direction
    def run(self, agent):
        raw_time_remaining = self.intercept_time - agent.time
        time_remaining = cap(raw_time_remaining, 0.001, 10.0)

        angle_from_ball = agent.me.forward.angle(self.shot_vector)

        car_to_ball = self.ball_location - agent.me.location
        
        towards_wall = distance_to_wall(self.ball_location)[1]

        if is_on_wall(agent.me.location, False):
            side_of_shot = sign(self.shot_vector.cross(towards_wall).dot(car_to_ball))
            shot_vector_perp = self.shot_vector.cross(towards_wall * side_of_shot)
        else:
            side_of_shot = sign(self.shot_vector.cross((0, 0, 1)).dot(car_to_ball))
            shot_vector_perp = self.shot_vector.cross((0, 0, side_of_shot))

        horizontal_dodge_offset = shot_vector_perp * cap(math.sin(angle_from_ball) * 10, -1, 1) * abs(length_by_angle(angle_from_ball)[1])
        self.dodge_point = self.ball_location - self.shot_vector * (95 + length_by_angle(angle_from_ball)[0]) + horizontal_dodge_offset

        car_to_ball = self.dodge_point - agent.me.location

        distance_to_ball = (self.ball_location - agent.me.location).magnitude()

        if is_on_wall(agent.me.location, False):
            distance = car_to_ball.magnitude()
            speed_required = distance / time_remaining
        else:
            distance = ((self.dodge_point.flatten() + towards_wall * self.dodge_point[2]) - agent.me.location).magnitude()
            speed_required = distance / time_remaining

        dodge_length = (self.dodge_point - self.ball_location).magnitude()

        time_until_hit = (distance / agent.me.velocity.magnitude()) if distance_to_ball > dodge_length else 0.1

        height_of_jump = cap(agent.me.local(self.dodge_point)[2], 60, 9999) if is_on_wall(agent.me.location, False) else 60

        int_jump_velocity = 291.667
        jump_acceleration = 1458.333374
        jump_stop_time = 0.2
        
        height_after_jump = int_jump_velocity * jump_stop_time + (1 / 2) * jump_acceleration * math.pow(jump_stop_time, 2)

        if height_after_jump < height_of_jump:
            velocity_after_jump = int_jump_velocity + jump_acceleration * jump_stop_time
            time_to_jump = jump_stop_time + (height_of_jump - height_after_jump) / velocity_after_jump
        else:
            fin_jump_velocity = math.sqrt(math.pow(int_jump_velocity, 2) + 2 * jump_acceleration * height_of_jump)
            time_to_jump = (fin_jump_velocity - int_jump_velocity) / jump_acceleration

        if not self.jumping:
            line_up_for_shot(self.ball_location, self.intercept_time, self.shot_vector, time_to_jump, 2).run(agent)

            if time_remaining < 0.01 or (speed_required - 2300) * time_remaining > 45 or not shot_valid(agent, self):
                agent.pop()
                if agent.me.airborne:
                    agent.push(recovery())
            elif time_remaining < time_to_jump:
                self.jumping = True 
        else:
            if (raw_time_remaining > 0.2 and not shot_valid(agent, self, 60)) or raw_time_remaining <= -0.8 or (not agent.me.airborne and self.counter > 0):
                agent.pop()
                agent.push(recovery())
            elif self.counter == 0 and raw_time_remaining > 0.05 and time_until_hit > 0.05:
                #Initial jump to get airborne + we hold the jump button for extra power as required
                agent.controller.jump = True
            elif self.counter < 3:
                #make sure we aren't jumping for at least 3 frames
                agent.controller.jump = False
                self.counter += 1
            elif raw_time_remaining <= 0.1 and raw_time_remaining > -0.7 and (distance < 40 or distance_to_ball < dodge_length):
                #dodge in the direction of the shot_vector
                agent.controller.jump = True
                if not self.dodging:
                    vector = agent.me.local(self.shot_vector)
                    self.p = -vector[0]
                    self.y = vector[1]
                    self.dodging = True
                #simulating a deadzone so that the dodge is more natural
                agent.controller.pitch = self.p if abs(self.p) > 0.2 else 0 
                agent.controller.yaw = self.y if abs(self.y) > 0.3 else 0

class flip():
    #Flip takes a vector in local coordinates and flips/dodges in that direction
    #cancel causes the flip to cancel halfway through, which can be used to half-flip
    def __init__(self, vector, cancel=False):
        self.vector = vector.normalize()
        self.pitch = abs(self.vector[0]) * -sign(self.vector[0])
        self.yaw = abs(self.vector[1]) * sign(self.vector[1])
        self.cancel = cancel
        #the time the jump began
        self.time = -1
        #keeps track of the frames the jump button has been released
        self.counter = 0
    def run(self, agent):
        if self.time == -1:
            elapsed = 0
            self.time = agent.time
        else:
            elapsed = agent.time - self.time
        if elapsed < 0.15:
            agent.controller.jump = True
        elif elapsed >=0.15 and self.counter < 3:
            agent.controller.jump = False
            self.counter += 1
        elif elapsed < 0.3 or (not self.cancel and elapsed < 0.9):
            agent.controller.jump = True
            agent.controller.pitch = self.pitch
            agent.controller.yaw = self.yaw
        else:
            agent.pop()
            agent.push(recovery())

class recovery():
    #Point towards our velocity vector and land upright, unless we aren't moving very fast
    #A vector can be provided to control where the car points when it lands
    def __init__(self, target=None):
        self.target = target
    def run(self, agent):
        if self.target != None:
            local_target = agent.me.local((self.target-agent.me.location).flatten())
        else:
            local_target = agent.me.local(agent.me.velocity.flatten())

        angles = default_orient(agent, local_target)
        agent.controller.throttle = 1

        angles_magnitude = angles[0] + angles[1] + angles[2]

        if angles_magnitude < 1 and not agent.me.doublejumped:
            agent.pop()
            #Wavedash recovery!
            agent.push(wavedash())

        if not agent.me.airborne:
            agent.pop()

class wavedash():
    # this routine will wavedash on recovery!
    def __init__(self):
        self.step = 0
    def run(self, agent):
        if agent.me.velocity.flatten().magnitude() > 100:
            target = agent.me.velocity.flatten().normalize()*100 + Vector3(0, 0, 50)
        else:
            target = agent.me.forward.flatten()*100 + Vector3(0, 0, 50)
        local_target = agent.me.local(target)
        default_orient(agent, local_target)
        if self.step < 6:
            self.step += 1
            if self.step < 3:
                agent.controller.jump = True
            else:
                agent.controller.jump = False
        else:
            if (agent.me.location + (agent.me.velocity * 0.2)).z < 5:
                agent.controller.jump = True
                agent.controller.pitch = -1
                agent.controller.yaw = agent.controller.roll = 0
                agent.pop()
            elif not agent.me.airborne or agent.me.doublejumped:
                agent.pop()