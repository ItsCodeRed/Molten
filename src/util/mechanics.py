from util.utils import *
from util.objects import *

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

class aerial():
    def __init__(self, ball_location, intercept_time, shot_vector, direction):
        self.ball_location = ball_location
        self.shot_vector = shot_vector
        self.target_location = self.ball_location - self.shot_vector * 170
        self.intercept_time = intercept_time
        self.direction = direction
        self.jump_threshold = 500
        self.launching = False
        self.jumping = -1
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
        raw_time_remaining = self.intercept_time - agent.time
        time_remaining = cap(raw_time_remaining, 0.001, 10.0)

        angle_from_ball = agent.me.forward.angle(self.shot_vector)
        angle_from_ball_sign = sign(agent.me.forward.anglesign(self.shot_vector))
        max_angle = get_max_angle(time_remaining, 9.11, agent.me.angular_velocity[2] * angle_from_ball_sign)

        self.target_location = self.ball_location - self.shot_vector * car_ball_collision_offset(cap(angle_from_ball - max_angle, 0, math.pi * 2))
        
        if self.jumping == -1:
            self.jumping = not agent.me.airborne

        if not self.launching and self.jumping:
            car_to_target = self.target_location - agent.me.location

            distance = car_to_target.magnitude()
            speed_required = distance / time_remaining

            line_up_for_shot(self.target_location, speed_required, self.intercept_time, self.shot_vector, self.direction, 1).run(agent)

            perdicted_location = agent.me.location.flatten() + agent.me.velocity.flatten() * time_remaining
            distance_off_mark = (self.target_location.flatten() - perdicted_location).magnitude()

            if time_remaining < 0.01 or not shot_valid(agent, self, threshold=150):
                agent.pop()
                if agent.me.airborne:
                    agent.push(recovery())
            elif distance_off_mark < 500:
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

            delta_x = self.target_location - xf
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
        time_remaining = self.intercept_time - time

        if not agent.me.airborne:
            ball_to_car = (self.ball_location - agent.me.location).flatten()
            car_final_vel = ball_to_car / time_remaining
            speed_diff = (car_final_vel - agent.me.velocity.flatten()).magnitude()

            T = self.intercept_time - time - speed_diff / 700
            xf = agent.me.location + car_final_vel * T + 0.5 * gravity * T ** 2
            vf = car_final_vel + gravity * T

            vf += agent.me.orientation.up * (2 * jump_speed + jump_acc * jump_max_duration)
            xf += agent.me.orientation.up * (jump_speed * (2 * T - jump_max_duration) + jump_acc * (
                    T * jump_max_duration - 0.5 * jump_max_duration ** 2))
        else:
            T = self.intercept_time - time
            xf = agent.me.location + agent.me.velocity * T + 0.5 * gravity * T ** 2
            vf = agent.me.velocity + gravity * T

        delta_x = self.ball_location - xf
        f = delta_x.normalize()
        phi = f.angle3D(agent.me.forward)
        turn_time = 0.7 * (2 * math.sqrt(phi / 9))

        tau1 = turn_time * cap(1 - 0.3 / phi, 0, 1)
        required_acc = (2 * delta_x.magnitude()) / ((T - tau1) ** 2)
        ratio = required_acc / boost_accel
        tau2 = T - (T - tau1) * math.sqrt(1 - cap(ratio, 0, 1))
        velocity_estimate = vf + boost_accel * (tau2 - tau1) * f
        boos_estimate = (tau2 - tau1) * 33.33
        enough_boost = boos_estimate < 0.95 * agent.me.boost
        enough_time = abs(ratio) < 0.9

        in_goal = abs(agent.me.location.y) > 5200

        ball_over_goal = self.ball_location.z > 800 and abs(self.ball_location.x) < 900 and abs(self.ball_location.y) > 4800

        return velocity_estimate.magnitude() < 0.9 * max_speed and enough_boost and enough_time and (not in_goal or not ball_over_goal)


class line_up_for_shot():
    def __init__(self, target_location, speed_required, intercept_time, direction_vector, time_before_jump, accuracy):
        self.direction_vector = direction_vector
        self.target_location = target_location
        self.speed_required = speed_required
        self.intercept_time = intercept_time
        self.time_before_jump = time_before_jump
        self.accuracy_multiplier = accuracy
    def run(self, agent):
        raw_time_remaining = self.intercept_time - agent.time
        time_remaining = cap(raw_time_remaining, 0.001, 10.0)

        car_to_target = self.target_location - agent.me.location

        wall_distance, towards_wall = distance_to_wall(self.target_location)

        wall_mod = 1

        if is_on_wall(self.target_location, True) and abs(self.target_location[0]) > 1000:
            if is_on_wall(agent.me.location, False):
                side_of_shot = sign(self.direction_vector.cross(towards_wall).dot(car_to_target))
                car_to_target_perp = car_to_target.cross(towards_wall * side_of_shot)

                distance = car_to_target.magnitude()
            else:
                self.target_location = self.target_location + towards_wall * (self.target_location[2] + wall_distance - 100)
                car_to_target = (self.target_location - agent.me.location).flatten()
                side_of_shot = sign(self.direction_vector.cross((0, 0, 1)).dot(car_to_target))
                car_to_target_perp = car_to_target.cross((0, 0, side_of_shot))

                distance = car_to_target.magnitude()
                wall_mod = 0.25
        else:
            side_of_shot = sign(self.direction_vector.cross((0, 0, 1)).dot(car_to_target))
            car_to_target_perp = car_to_target.cross((0, 0, side_of_shot))

            distance = car_to_target.flatten().magnitude()

            if is_on_wall(agent.me.location, False) and abs(agent.me.location[0]) > 1000:
                self.target_location = self.target_location.flatten()

        angle_from_shot = car_to_target.angle(self.direction_vector)
        angle_from_ball = agent.me.forward.angle(self.direction_vector)
        
        adjustment = (angle_from_shot * distance / (3 - self.accuracy_multiplier + angle_from_ball / math.pi)) * wall_mod
        adjustment *= cap(time_remaining - self.time_before_jump * 2, 0, self.time_before_jump * 2) / (self.time_before_jump * 2 + 0.001)
        adjusted_target = self.target_location + ((car_to_target_perp.normalize() * adjustment) if not agent.me.airborne else 0)

        if not is_on_wall(self.target_location, True):
            if abs(adjusted_target.x) > 4096 + distance_to_wall(agent.me.location)[0]:
                adjusted_target = adjusted_target + car_to_target_perp.normalize() * (((4000 + distance_to_wall(agent.me.location)[0]) * sign(adjusted_target.x) - adjusted_target.x) / car_to_target_perp.normalize().x)
            if abs(adjusted_target.y) > 5120 + distance_to_wall(agent.me.location)[0]:
                adjusted_target = adjusted_target + car_to_target_perp.normalize() * (((5000 + distance_to_wall(agent.me.location)[0]) * sign(adjusted_target.y) - adjusted_target.y) / car_to_target_perp.normalize().y)

        boost_score = 9999
        best_boost_location = 0
        for boost in agent.pads + agent.boosts:
            distance_to_boost = (boost.location - agent.me.location).magnitude()
            angle_to_boost = (boost.location - agent.me.location).angle(adjusted_target - agent.me.location)
            if distance / (1 + agent.me.boost / 5) > angle_to_boost * distance_to_boost < boost_score and distance > 500 and angle_to_boost < 0.5 and boost.active and time_remaining > self.time_before_jump * 6:
                best_boost_location = boost.location
                boost_score = angle_to_boost * distance_to_boost

        if abs(agent.me.location.y) > 5020 and abs(agent.me.location.x) < 900:
            adjusted_target.x = cap(adjusted_target.x, -700, 700)
            adjusted_target.y = cap(adjusted_target.y, -5000, 5000)
            adjusted_target.z = 100

            local_target = agent.me.local(adjusted_target - agent.me.location)
        elif best_boost_location != 0:
            local_target = agent.me.local(best_boost_location - agent.me.location)
        else:
            local_target = agent.me.local(adjusted_target - agent.me.location)

        vel_in_direction = math.cos(car_to_target.angle(agent.me.velocity)) * agent.me.velocity.magnitude()
        space_to_flip = distance - vel_in_direction - 600 > self.speed_required * self.time_before_jump

        angles = default_orient(agent, local_target)
        default_throttle(agent, self.speed_required)

        if not is_on_wall(self.target_location, True) and not is_on_wall(agent.me.location, False) and space_to_flip and abs(angles[1]) > 2.8 and not agent.me.airborne:
            if agent.me.local(agent.me.velocity)[0] < 0:
                agent.push(flip(Vector3(-1, 0, 0), True))
            else:
                agent.controller.throttle = -1

        agent.controller.boost = False if abs(angles[1]) > 0.3 or agent.me.airborne else agent.controller.boost
        agent.controller.handbrake = True if abs(angles[1]) > 2 else agent.controller.handbrake

        agent.line(agent.me.location, self.target_location)
        agent.line(self.target_location-Vector3(0, 0, 100), self.target_location + Vector3(0, 0, 100), [255, 0, 0])
        agent.line(adjusted_target-Vector3(0, 0, 100), adjusted_target+Vector3(0, 0, 100), [0, 255, 0])
        agent.line(self.target_location, self.target_location + self.direction_vector * 200, [0, 0, 255])

class shoot():
    def __init__(self, ball_location, intercept_time, shot_vector, direction):
        self.ball_location = ball_location
        self.intercept_time = intercept_time
        self.shot_vector = shot_vector
        dodge_shot_angle = math.acos(shot_vector.flatten().magnitude()) + 0.45
        self.dodge_vector = shot_vector.flatten().normalize() * math.cos(dodge_shot_angle) + Vector3(0,0,1) * math.sin(dodge_shot_angle)
        self.dodge_point = self.ball_location - (self.shot_vector * 170)
        self.direction = direction
        self.time_of_jump = -1
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

        time_to_jump = find_jump_time(cap(self.dodge_point[2] - agent.me.location[2], 1, 300))
        time_until_dodge = cap(time_remaining - 0.1, 0.001, 10.0)

        car_to_ball = self.ball_location - agent.me.location

        angle_from_ball = agent.me.forward.angle(self.shot_vector)
        angle_from_ball_sign = sign(agent.me.forward.anglesign(self.shot_vector))
        max_angle = get_max_angle(time_until_dodge, 9.11, agent.me.angular_velocity[2] * angle_from_ball_sign)

        self.dodge_point = self.ball_location - self.shot_vector * car_ball_collision_offset(cap(angle_from_ball - max_angle, 0, math.pi * 2))
        intercept_point = self.dodge_point + self.shot_vector * 94.41

        car_to_dodge_point = self.dodge_point - agent.me.location
        distance = car_to_dodge_point.flatten().magnitude()
        speed_required = distance / time_remaining

        jump_acc_time = cap(time_until_dodge, 0, jump_max_duration)

        up_during_jump = (agent.me.up * 2 + self.shot_vector.cross(self.shot_vector.cross(Vector3(0, 0, 1))).normalize()).normalize()

        perdicted_location = perdict_car_location(agent.me, time_until_dodge) + \
            agent.me.up * jump_speed * time_until_dodge + \
            up_during_jump * jump_acc * jump_acc_time * (time_until_dodge - 0.5 * jump_acc_time) + \
            agent.me.velocity.flatten() * 0.1 + \
            self.shot_vector.flatten().normalize() * 60

        perdicted_hitbox = agent.me.hitbox
        perdicted_hitbox.location = perdicted_location
        perdicted_hitbox.orientation = Matrix3(
            self.shot_vector,
            self.shot_vector.cross(Vector3(0, 0, 1)).normalize(),
            self.shot_vector.cross(self.shot_vector.cross(Vector3(0, 0, 1))).normalize()
        )
        perdicted_hitbox.render(agent, [255,0,0])
        on_target = (self.dodge_point - perdicted_location).magnitude() <  50
        missing = not perdicted_hitbox.intersect_ball(self.ball_location)

        if not self.jumping:
            line_up_for_shot(self.dodge_point - self.shot_vector.flatten().normalize() * 60, speed_required, self.intercept_time, self.shot_vector, time_to_jump, 1).run(agent)

            if time_until_dodge < time_to_jump * 0.8 or (speed_required - 2300) * time_remaining > 45 or not shot_valid(agent, self):
                agent.pop()
                if agent.me.airborne:
                    agent.push(recovery())
            elif not missing and (time_until_dodge < time_to_jump or (time_until_dodge < 1.25 and on_target)):
                self.jumping = True 
                if self.time_of_jump == -1:
                    elapsed = 0
                    self.time_of_jump = agent.time
        else:
            elapsed = agent.time - self.time_of_jump
            default_orient(agent, agent.me.local(self.dodge_vector))
            # default_orient(agent, agent.me.local(agent.me.forward.flatten().normalize() * self.shot_vector.flatten().magnitude() + Vector3(0, 0, self.shot_vector.z)))

            if (raw_time_remaining > 0.1 and not shot_valid(agent, self, 60)) or raw_time_remaining <= -0.1 or (not agent.me.airborne and self.counter > 0):
                agent.pop()
                agent.push(recovery())
            elif self.counter == 0 and elapsed < cap(time_to_jump, 0, 0.2) and raw_time_remaining > 0.11:
                #Initial jump to get airborne + we hold the jump button for extra power as required
                agent.controller.jump = True
            elif self.counter < 2:
                #make sure we aren't jumping for at least 3 frames
                agent.controller.jump = False
                self.counter += 1
            elif raw_time_remaining <= 0.1 and raw_time_remaining > -0.9:
                #dodge in the direction of the shot_vector
                vector = agent.me.local(self.shot_vector)
                #simulating a deadzone so that the dodge is more natural
                vector[0] = vector[0] if abs(vector[0]) > 0.2 else 0 
                vector[1] = vector[1] if abs(vector[1]) > 0.3 else 0
                agent.pop()
                agent.push(flip(vector))

class pop_up():
    def __init__(self, ball_location, intercept_time, shot_vector, direction):
        self.ball_location = ball_location
        self.intercept_time = intercept_time
        self.shot_vector = shot_vector
        self.intercept = self.ball_location - (self.shot_vector * 170)
        self.direction = direction
    def run(self, agent):
        raw_time_remaining = self.intercept_time - agent.time
        time_remaining = cap(raw_time_remaining, 0.001, 10.0)

        angle_from_ball = agent.me.forward.angle(self.shot_vector)

        self.intercept = self.ball_location - self.shot_vector * ((car_ball_collision_offset(angle_from_ball) - 94.41) * 0.8 + 94.41)

        distance = (self.intercept - agent.me.location).flatten().magnitude()
        speed_required = distance / time_remaining

        line_up_for_shot(self.intercept, speed_required, self.intercept_time, self.shot_vector, 0.2, 1).run(agent)

        if (speed_required - 2300) * time_remaining > 45 or not shot_valid(agent, self) or raw_time_remaining <= -0.3:
            agent.pop()
            if agent.me.airborne:
                agent.push(recovery())

class double_jump():
    def __init__(self, ball_location, intercept_time, shot_vector, direction):
        self.ball_location = ball_location
        self.intercept_time = intercept_time
        self.shot_vector = shot_vector
        self.dodge_point = self.ball_location - self.shot_vector * 150
        self.direction = direction
        self.jumping = False
        self.counter = 0
        self.time_of_jump = -1
    def update(self, ball_location, intercept_time, shot_vector, direction):
        self.ball_location = ball_location
        self.shot_vector = shot_vector
        self.dodge_point = self.ball_location - self.shot_vector * 150
        self.intercept_time = intercept_time
        self.direction = direction
    def run(self, agent):
        raw_time_remaining = self.intercept_time - agent.time
        time_remaining = cap(raw_time_remaining, 0.001, 10.0)

        car_to_ball = self.dodge_point - agent.me.location
        distance = car_to_ball.flatten().magnitude()
        speed_required = distance / time_remaining

        angle_from_shot = car_to_ball.angle(self.shot_vector)

        intercept_point = self.dodge_point + self.shot_vector * 95

        time_to_jump = find_jump_time(cap(self.dodge_point[2] - agent.me.location[2], 0, 500), True)

        perdicted_location = perdict_car_location(agent.me, time_remaining) + \
            agent.me.up * jump_speed * time_remaining + \
            agent.me.up * jump_acc * jump_max_duration * (time_remaining - 0.5 * jump_max_duration) + \
            agent.me.up * jump_speed * (time_remaining - jump_max_duration)

        perdicted_hitbox = agent.me.hitbox
        perdicted_hitbox.location = perdicted_location
        perdicted_hitbox.orientation = Matrix3(
            self.shot_vector,
            self.shot_vector.cross(Vector3(0, 0, 1)).normalize(),
            self.shot_vector.cross(self.shot_vector.cross(Vector3(0, 0, 1))).normalize()
        )
        perdicted_hitbox.render(agent, [255,0,0])
        missing = not perdicted_hitbox.intersect_ball(self.ball_location)

        if not self.jumping:
            line_up_for_shot(self.dodge_point, speed_required, self.intercept_time, self.shot_vector, time_to_jump, 1).run(agent)

            if time_remaining < time_to_jump * 0.9 or (speed_required - 2300) * time_remaining > 45 or not shot_valid(agent, self):
                agent.pop()
                if agent.me.airborne:
                    agent.push(recovery())
            elif not missing:
                self.jumping = True 
                if self.time_of_jump == -1:
                    elapsed = 0
                    self.time_of_jump = agent.time
        else:
            elapsed = agent.time - self.time_of_jump
            if elapsed > 0.2 and self.counter > 3:
                default_orient(agent, agent.me.local(self.shot_vector))
            else:
                default_orient(agent, agent.me.local(agent.me.forward))

            if (raw_time_remaining > 0.2 and not shot_valid(agent, self, 60)) or raw_time_remaining <= -0.3 or (not agent.me.airborne and self.counter > 0):
                agent.pop()
                agent.push(recovery())
            elif self.counter == 0 and elapsed < 0.2:
                #Initial jump to get airborne + we hold the jump button for extra power as required
                agent.controller.jump = True
            elif self.counter < 2:
                #make sure we aren't jumping for at least 3 frames
                agent.controller.jump = False
                self.counter += 1
            elif self.counter < 4:
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

        height_of_jump, towards_wall = distance_to_wall(self.ball_location)
        self.shot_vector = self.shot_vector.flatten_by_vector(towards_wall)

        angle_from_ball = agent.me.forward.angle3D(self.shot_vector)

        dodge_length = car_ball_collision_offset(angle_from_ball)
        self.dodge_point = self.ball_location - self.shot_vector * dodge_length
        best_point = self.dodge_point + self.shot_vector * 95

        car_to_ball = self.ball_location - agent.me.location
        car_to_dodge_point = self.dodge_point - agent.me.location

        distance_to_ball = (self.ball_location - agent.me.location).magnitude()
        if is_on_wall(agent.me.location, False):
            distance = car_to_dodge_point.magnitude()
        else:
            distance = (self.dodge_point.flatten() + towards_wall * (self.dodge_point[2] + height_of_jump - 100) - agent.me.location).magnitude()
        speed_required = distance / time_remaining

        jump_acc_time = cap(time_remaining, 0, jump_max_duration)

        perdicted_location = perdict_car_location(agent.me, time_remaining) + \
            agent.me.up * jump_speed * time_remaining + \
            agent.me.up * jump_acc * jump_acc_time * (time_remaining - 0.5 * jump_acc_time)
        dodge_to_perdicted = self.dodge_point - perdicted_location

        adjusted_target = self.dodge_point + dodge_to_perdicted.flatten_by_vector(towards_wall) if is_on_wall(agent.me.location, False) and height_of_jump > 120 else self.dodge_point

        perdicted_hitbox = agent.me.hitbox
        perdicted_hitbox.location = perdicted_location
        perdicted_hitbox.orientation = Matrix3(
            self.shot_vector,
            self.shot_vector.cross(agent.me.up).normalize(),
            self.shot_vector.cross(self.shot_vector.cross(agent.me.up)).normalize()
        )
        perdicted_hitbox.render(agent, [255,0,0])
        missing = not perdicted_hitbox.intersect_ball(self.ball_location)

        if not self.jumping:
            line_up_for_shot(adjusted_target, speed_required, self.intercept_time, self.shot_vector, 0.3, 2).run(agent)

            if raw_time_remaining < -0.2 or (speed_required - 2300) * time_remaining > 45 or not shot_valid(agent, self):
                agent.pop()
                if agent.me.airborne:
                    agent.push(recovery())
            elif not missing and height_of_jump > 120:
                self.jumping = True 
        else:
            if (raw_time_remaining > 0.2 and not shot_valid(agent, self, 60)) or raw_time_remaining <= -1 or (not agent.me.airborne and self.counter > 0):
                agent.pop()
                agent.push(recovery())
            elif self.counter == 0 and raw_time_remaining > 0.05:
                #Initial jump to get airborne + we hold the jump button for extra power as required
                agent.controller.jump = True
            elif self.counter < 3:
                #make sure we aren't jumping for at least 3 frames
                agent.controller.jump = False
                self.counter += 1
            elif raw_time_remaining <= 0.1 and raw_time_remaining > -0.9:
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

class short_shot():
    def __init__(self,target):
        self.target = target
    def run(self,agent):
        car_to_ball,distance = (agent.ball.location - agent.me.location).normalize(True)
        ball_to_target = (self.target - agent.ball.location).normalize()

        ball_to_our_goal = (agent.friend_goal.location - agent.ball.location)

        relative_velocity = car_to_ball.dot(agent.me.velocity-agent.ball.velocity)
        if relative_velocity != 0.0:
            eta = cap(distance / cap(relative_velocity,400,2300),0.0, 1.5)
        else:
            eta = 1.5

        #If we are approaching the ball from the wrong side the car will try to only hit the very edge of the ball
        left_vector = car_to_ball.cross((0,0,1))
        right_vector = car_to_ball.cross((0,0,-1))
        target_vector = -ball_to_target.clamp(left_vector, right_vector)
        final_target = agent.ball.location + (target_vector*(distance/2))

        #Some extra adjustment to the final target to ensure it's inside the field and we don't try to dirve through any goalposts to reach it
        if abs(agent.me.location[1]) > 5000:
            if abs(final_target[0]) < 800:
                final_target[0] = cap(final_target[0],-800,800)
            else:
                final_target[1] = cap(final_target[1],-5100,5100)
        
        agent.line(final_target-Vector3(0,0,100),final_target+Vector3(0,0,100),[255,255,255])
        
        angles = default_orient(agent, agent.me.local(final_target-agent.me.location))
        default_throttle(agent, 2300 if distance > 1600 else 2300-cap(1600*abs(angles[1]),0,2050))
        agent.controller.boost = False if agent.me.airborne or abs(angles[1]) > 0.3 else agent.controller.boost
        agent.controller.handbrake = True if abs(angles[1]) > 2.3 else agent.controller.handbrake

        if abs(angles[1]) < 0.05 and (eta < 0.45 or distance < 150):
            agent.pop()
            agent.push(flip(agent.me.local(car_to_ball)))
        if eta > 1 or ball_to_our_goal.magnitude() < 500 or agent.rotation_index != 0:
            agent.pop()

class goto():
    def __init__(self, location, direction, speed):
        self.target_location = location
        self.direction_vector = direction.normalize()
        self.speed = speed
    def update(self, location, direction, speed):
        self.target_location = location
        self.direction_vector = direction.normalize()
        self.speed = speed
    def run(self, agent):
        car_to_target = self.target_location - agent.me.location

        wall_distance, towards_wall = distance_to_wall(self.target_location)

        side_of_shot = sign(self.direction_vector.cross((0, 0, 1)).dot(car_to_target))
        car_to_target_perp = car_to_target.cross((0, 0, side_of_shot))

        distance = car_to_target.flatten().magnitude()
        speed_required = self.speed * cap(distance / 1000, 0, 1)

        if is_on_wall(agent.me.location, False) and abs(agent.me.location[0]) > 1000:
            self.target_location = self.target_location.flatten()

        angle_from_shot = car_to_target.angle(self.direction_vector)
        angle_from_ball = agent.me.forward.angle(self.direction_vector)
        
        adjustment = (angle_from_shot * distance / (2 + angle_from_ball / math.pi))
        adjusted_target = self.target_location + ((car_to_target_perp.normalize() * adjustment) if not agent.me.airborne else 0)

        if not is_on_wall(self.target_location, True):
            if abs(adjusted_target.x) > 4096 + distance_to_wall(agent.me.location)[0]:
                adjusted_target = adjusted_target + car_to_target_perp.normalize() * (((4000 + distance_to_wall(agent.me.location)[0]) * sign(adjusted_target.x) - adjusted_target.x) / car_to_target_perp.normalize().x)
            if abs(adjusted_target.y) > 5120 + distance_to_wall(agent.me.location)[0]:
                adjusted_target = adjusted_target + car_to_target_perp.normalize() * (((5000 + distance_to_wall(agent.me.location)[0]) * sign(adjusted_target.y) - adjusted_target.y) / car_to_target_perp.normalize().y)

        boost_score = 9999
        best_boost_location = 0
        for boost in agent.pads + agent.boosts:
            distance_to_boost = (boost.location - agent.me.location).magnitude()
            angle_to_boost = (boost.location - agent.me.location).angle(adjusted_target - agent.me.location)
            if distance / (1 + agent.me.boost / 5) > angle_to_boost * distance_to_boost < boost_score and distance > 500 and angle_to_boost < 0.5 and boost.active:
                best_boost_location = boost.location
                boost_score = angle_to_boost * distance_to_boost

        if abs(agent.me.location.y) > 5020 and abs(agent.me.location.x) < 900:
            adjusted_target.x = cap(adjusted_target.x, -700, 700)
            adjusted_target.y = cap(adjusted_target.y, -5000, 5000)
            adjusted_target.z = 100

            local_target = agent.me.local(adjusted_target - agent.me.location)
        elif best_boost_location != 0:
            local_target = agent.me.local(best_boost_location - agent.me.location)
        else:
            local_target = agent.me.local(adjusted_target - agent.me.location)

        vel_in_direction = math.cos(car_to_target.angle(agent.me.velocity)) * agent.me.velocity.magnitude()

        angles = default_orient(agent, local_target)
        default_throttle(agent, speed_required)

        agent.controller.boost = False if abs(angles[1]) > 0.3 or agent.me.airborne else agent.controller.boost
        agent.controller.handbrake = True if abs(angles[1]) > 2 else agent.controller.handbrake

        agent.line(agent.me.location, self.target_location)
        agent.line(self.target_location-Vector3(0, 0, 100), self.target_location + Vector3(0, 0, 100), [255, 0, 0])
        agent.line(adjusted_target-Vector3(0, 0, 100), adjusted_target+Vector3(0, 0, 100), [0, 255, 0])
        agent.line(self.target_location, self.target_location + self.direction_vector * 200, [0, 0, 255])


class kickoff():
    def __init__(self, side):
        self.has_speed_flipped = False
        self.side = side
    def run(self, agent):
        if not agent.kickoff:
            agent.pop()

        car_to_ball = agent.ball.location - agent.me.location
        distance = car_to_ball.magnitude()
        speed = agent.me.velocity.magnitude()

        corner_kickoff = abs(agent.me.location.x) > 1500
        team = -side(agent.team)

        steer = self.side * team if corner_kickoff else -self.side * team
        local_car_to_ball = agent.me.local(car_to_ball)

        speed_flip_vector = Vector3(1/math.sqrt(2), -steer/math.sqrt(2), 0)

        speed_threshold = 690 if corner_kickoff else 700

        if speed > speed_threshold and not self.has_speed_flipped:
            self.has_speed_flipped = True
            agent.push(flip(speed_flip_vector, True, True))
        elif speed > 500 and not self.has_speed_flipped:
            agent.controller.steer = steer
            default_throttle(agent, 2300)
        elif distance < 800:
            agent.push(flip(local_car_to_ball))
        else:
            default_orient(agent, local_car_to_ball)
            default_throttle(agent, 2300)

class flip():
    #Flip takes a vector in local coordinates and flips/dodges in that direction
    #cancel causes the flip to cancel halfway through, which can be used to half-flip
    def __init__(self, vector, cancel=False, boosting=False):
        self.vector = vector.normalize()
        self.pitch = -self.vector[0]
        self.yaw = self.vector[1]
        self.cancel = cancel
        self.boosting = boosting
        self.jumped = False
        #the time the jump began
        self.time = -1
        #keeps track of the frames the jump button has been released
        self.counter = 0
    def run(self, agent):
        if self.time == -1:
            elapsed = 0
            self.time = agent.time
            self.jumped = agent.me.airborne
        else:
            elapsed = agent.time - self.time
        if elapsed < 0.15 and not self.jumped:
            agent.controller.jump = True
        elif elapsed >=0.15 and self.counter < 3 and not self.jumped:
            agent.controller.jump = False
            self.counter += 1
        elif elapsed < 0.3 or (not self.cancel and elapsed < 0.9):
            agent.controller.jump = True
            agent.controller.pitch = self.pitch
            agent.controller.yaw = self.yaw
        else:
            agent.pop()
            agent.push(recovery(None, self.boosting))
        if self.boosting:
            agent.controller.boost = agent.me.local(agent.me.forward).angle3D(self.vector) < math.pi


class recovery():
    #Point towards our velocity vector and land upright, unless we aren't moving very fast
    #A vector can be provided to control where the car points when it lands
    def __init__(self, target=None, boosting=False):
        self.target = target
        self.boosting = boosting
    def run(self, agent):
        if self.target != None:
            local_target = agent.me.local((self.target-agent.me.location).flatten())
        else:
            local_target = agent.me.local(agent.me.velocity.flatten())

        angles = default_orient(agent, local_target)
        agent.controller.throttle = 1
        agent.controller.boost = self.boosting

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
        if self.step < 6 and not agent.me.airborne:
            self.step += 1
            if self.step < 3:
                agent.controller.jump = True
            else:
                agent.controller.jump = False
        else:
            if agent.me.location.z + agent.me.velocity.z * 0.2 < 5:
                agent.controller.jump = True
                agent.controller.pitch = -1
                agent.controller.yaw = agent.controller.roll = 0
                agent.pop()
            elif not agent.me.airborne or agent.me.doublejumped:
                agent.pop()