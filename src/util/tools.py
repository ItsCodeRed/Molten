from util.mechanics import *
from util.utils import *
import math

#This file is for small utilities for math and movement

def find_hits(agent, targets):
    #find_hits looks into the future and finds all future hits that the car can reach in time
    #and that could be shot between the targets provided
    hits = {name:[] for name in targets}
    struct = agent.get_ball_prediction_struct()

    i = 15
    while i < struct.num_slices:
        #Gather some data about the slice
        intercept_time = struct.slices[i].game_seconds
        time_remaining = intercept_time - agent.time
        if time_remaining > 0:
            ball_location = Vector3(struct.slices[i].physics.location)
            ball_velocity = Vector3(struct.slices[i].physics.velocity)

            if abs(ball_location[1]) > 5250:
                break #stop searching - a goal has been scored

            #determine the next slice we will look at, based on ball velocity (slower ball needs fewer slices)
            i += 15 - cap(int(ball_velocity.magnitude()//10), 0, 13)

            car_to_ball = ball_location - agent.me.location

            wall_distance, towards_wall = distance_to_wall(ball_location)

            if is_on_wall(ball_location, True) and abs(ball_location[0]) > 1500:
                if is_on_wall(agent.me.location, False):
                    distance = car_to_ball.magnitude()
                    direction = car_to_ball.normalize()
                else:
                    distance = ((ball_location.flatten() + towards_wall * ball_location[2]) - agent.me.location).magnitude()
                    direction = ((ball_location.flatten() + towards_wall * ball_location[2]) - agent.me.location).normalize()
            else:
                distance = car_to_ball.flatten().magnitude()
                direction = car_to_ball.normalize()

            goal_to_ball = agent.foe_goal.location - ball_location
            distance_to_goal = goal_to_ball.magnitude()
            car_final_vel = car_to_ball / time_remaining
            goal_speed = (car_final_vel - ball_velocity).magnitude()

            time_to_jump = find_jump_time(cap(ball_location[2], 0, 480), ball_location[2] >= 300) if ball_location[2] < 600 else 0.2
            estimated_time, forwards = eta(agent.me, ball_location, direction, distance)

            direction = car_to_ball.normalize()

            if estimated_time < time_remaining and time_remaining > time_to_jump:
                for pair in targets:
                    #First we correct the target coordinates to account for the ball's radius
                    #If swapped == True, the shot isn't possible because the ball wouldn't fit between the targets
                    left, right, swapped = post_correction(ball_location, targets[pair][0], targets[pair][1])
                    if not swapped:
                        #Now we find the easiest direction to hit the ball in order to land it between the target points
                        left_vector = (left - ball_location).normalize()
                        right_vector = (right - ball_location).normalize()
                        best_shot_vector = direction.clamp(left_vector, right_vector)

                        car_angle = car_to_ball.angle(best_shot_vector)
                        ball_angle = ball_velocity.angle(best_shot_vector)
                        s = cap(math.cos(car_angle / (1 + time_remaining)), 0, 1) * goal_speed + cap(math.cos(ball_angle), 0, 1) * ball_velocity.magnitude() # shot speed
                        g = 650 # gravity
                        x = ((left + right) / 2 - ball_location).flatten().magnitude() # horizontal offset from ball to target
                        y = ((left + right) / 2).z - ball_location.z # vertical offset from ball to target
                        
                        if s**4 - g * (g * x**2 + 2 * y * s**2) < 0:
                            shot_angle = 0
                        else:
                            shot_angle_1 = math.atan2(s**2 + math.sqrt(s**4 - g * (g * x**2 + 2 * y * s**2)), g * x)
                            shot_angle_2 = math.atan2(s**2 - math.sqrt(s**4 - g * (g * x**2 + 2 * y * s**2)), g * x)

                            shot_angle = shot_angle_1 if abs(shot_angle_1) < abs(shot_angle_2) else shot_angle_2

                        best_shot_vector = best_shot_vector.flatten().normalize() * math.cos(shot_angle) + Vector3(0,0,1) * math.sin(shot_angle)
                        best_shot_vector = (best_shot_vector * s - ball_velocity / 2).normalize()

                        if ball_location.z - best_shot_vector.z * 170 < 100:
                            best_shot_vector.z = (ball_location.z - 100) / 170
                        best_shot_vector = best_shot_vector.normalize()
                    
                        #Check to make sure our approach is inside the field
                        if in_field(ball_location - (200*best_shot_vector), 1):
                            #The slope represents how close the car is to the chosen vector, higher = better
                            #A slope of 1.0 would mean the car is 45 degrees off
                            slope = find_slope(best_shot_vector, car_to_ball)
                            if forwards:
                                if is_on_wall(ball_location, True) and abs(ball_location[0]) > 1000:
                                    hits[pair].append(wall_hit(ball_location, intercept_time, best_shot_vector.flatten_by_vector(towards_wall), 1))
                                elif ball_location[2] < 120 and distance_to_goal > 4000 and goal_speed > 2000:
                                    hits[pair].append(pop_up(ball_location, intercept_time, best_shot_vector, 1))
                                elif ball_location[2] < 300:
                                    hits[pair].append(shoot(ball_location, intercept_time, best_shot_vector, 1))
                                elif ball_location[2] - best_shot_vector[2] * 170 < 500:
                                    hits[pair].append(double_jump(ball_location, intercept_time, best_shot_vector, 1))
                                else:
                                    aerial_attempt = aerial(ball_location, intercept_time, best_shot_vector, 1)
                                    if aerial_attempt.is_viable(agent, agent.time):
                                        hits[pair].append(aerial_attempt)
                            else:
                                if ball_location[2] <= 300:
                                    hits[pair].append(shoot(ball_location, intercept_time, best_shot_vector, -1))
        else:
            i += 1
    return hits

def attack(agent):
    if len(agent.stack) < 1:
        ball_to_me = agent.ball.location - agent.me.location
        targets = {"goal":(agent.foe_goal.left_post, agent.foe_goal.right_post)}
        shots = find_hits(agent, targets)
        if len(shots["goal"]) > 0:
            agent.push(shots["goal"][0])
        else:
            agent.push(short_shot(agent.foe_goal.location))
    elif isinstance(agent.stack[-1], goto):
        agent.pop()

def save(agent):
    if len(agent.stack) < 1:
        ball_to_me = agent.ball.location - agent.me.location
        upfield_left = Vector3(-side(agent.team) * 4096, agent.ball.location.y - side(agent.team) * 2000, 0)
        upfield_right = Vector3(side(agent.team) * 4096, agent.ball.location.y - side(agent.team) * 2000, 0)
        targets = {"goal":(agent.foe_goal.left_post, agent.foe_goal.right_post), "upfield":(upfield_left, upfield_right)}
        shots = find_hits(agent, targets)
        if len(shots["upfield"]) > 0:
            agent.push(shots["upfield"][0])
        elif len(shots["goal"]) > 0:
            agent.push(shots["goal"][0])
        else:
            agent.push(short_shot(agent.foe_goal.location))
    elif isinstance(agent.stack[-1], goto):
        agent.pop()