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

            if is_on_wall(ball_location, True) and abs(ball_location[0]) > 1000:
                if is_on_wall(agent.me.location, False):
                    distance = car_to_ball.magnitude()
                    direction = car_to_ball.normalize()
                else:
                    distance = ((ball_location.flatten() + towards_wall * (ball_location[2] + wall_distance - 100)) - agent.me.location).flatten().magnitude()
                    direction = ((ball_location.flatten() + towards_wall * (ball_location[2] + wall_distance - 100)) - agent.me.location).normalize()
            else:
                distance = car_to_ball.flatten().magnitude()
                direction = car_to_ball.normalize()

            time_to_jump = find_jump_time(cap(ball_location[2], 0, 480), ball_location[2] >= 300) if ball_location[2] < 600 else 0.2
            estimated_time = eta(agent.me, ball_location, direction, distance)

            if estimated_time < time_remaining and time_remaining > time_to_jump:
                for pair in targets:
                    #First we correct the target coordinates to account for the ball's radius
                    #If swapped == True, the shot isn't possible because the ball wouldn't fit between the targets
                    left, right, swapped = post_correction(ball_location, targets[pair][0], targets[pair][1])
                    if not swapped:
                        #Now we find the easiest direction to hit the ball in order to land it between the target points
                        left_vector = (left - ball_location)
                        right_vector = (right - ball_location)
                        best_shot_vector, target_location = direction.clamp3D(left_vector, right_vector, True)

                        car_final_vel = car_to_ball / time_remaining

                        difference_in_vel = (car_final_vel - agent.me.velocity).magnitude()
                        
                        shot_speed = (best_shot_vector * ((car_final_vel - ball_velocity).magnitude() + (500 if 130 < ball_location[2] < 300 else 0)) * 4 + ball_velocity).magnitude()

                        shot_angle = find_shot_angle(shot_speed, target_location.flatten().magnitude(), target_location.z)
                        best_shot_vector = best_shot_vector.flatten().normalize() * math.cos(shot_angle) + Vector3(0,0,1) * math.sin(shot_angle)
                        best_shot_vector = (best_shot_vector * shot_speed - ball_velocity).normalize()
                        flattened = ball_location.z - best_shot_vector.z * 170 < 94.41
                        if flattened:
                            best_shot_vector.z = (ball_location.z - 94.41) / 170
                            best_shot_vector = best_shot_vector.normalize()
                    
                        #Check to make sure our approach is inside the field
                        if in_field(ball_location - (200*best_shot_vector), 1):
                            #The slope represents how close the car is to the chosen vector, higher = better
                            #A slope of 1.0 would mean the car is 45 degrees off
                            slope = find_slope(best_shot_vector, car_to_ball)
                            if is_on_wall(ball_location, True) and abs(ball_location[0]) > 1000:
                                hits[pair].append(wall_hit(ball_location, intercept_time, best_shot_vector.flatten_by_vector(towards_wall), 1))
                            elif ball_location[2] < 120 and flattened and (car_final_vel - ball_velocity).magnitude() > 2500:
                                hits[pair].append(pop_up(ball_location, intercept_time, best_shot_vector, 1))
                            elif ball_location[2] - best_shot_vector[2] * 150 < 300:
                                hits[pair].append(shoot(ball_location, intercept_time, best_shot_vector, 1))
                            elif ball_location[2] - best_shot_vector[2] * 150 < 500:
                                hits[pair].append(double_jump(ball_location, intercept_time, best_shot_vector, 1))
                            else:
                                aerial_attempt = aerial(ball_location, intercept_time, best_shot_vector, 1)
                                if aerial_attempt.is_viable(agent, agent.time):
                                    hits[pair].append(aerial_attempt)
        else:
            i += 1
    return hits

def attack(agent):
    if len(agent.stack) < 1:
        ball_to_me = agent.ball.location - agent.me.location
        targets = {"goal":(agent.foe_goal.left_post - Vector3(0, 0, 300), agent.foe_goal.right_post + Vector3(0, 0, 300))}
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
        upfield_right = Vector3(side(agent.team) * 4096, agent.ball.location.y - side(agent.team) * 2000, 1000)
        targets = {"goal":(agent.foe_goal.left_post - Vector3(0, 0, 300), agent.foe_goal.right_post + Vector3(0, 0, 300)), "upfield":(upfield_left, upfield_right)}
        shots = find_hits(agent, targets)
        if len(shots["upfield"]) > 0:
            agent.push(shots["upfield"][0])
        elif len(shots["goal"]) > 0:
            agent.push(shots["goal"][0])
        else:
            agent.push(short_shot(agent.foe_goal.location))
    elif isinstance(agent.stack[-1], goto):
        agent.pop()