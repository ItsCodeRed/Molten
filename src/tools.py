from mechanics import *

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
            ball_velocity = Vector3(struct.slices[i].physics.location)

            if abs(ball_location[1]) > 5250:
                break #stop searching - a goal has been scored

            #determine the next slice we will look at, based on ball velocity (slower ball needs fewer slices)
            i += 15 - cap(int(ball_velocity.magnitude()//10), 0, 13)

            car_to_ball = ball_location - agent.me.location

            towards_wall = distance_to_wall(ball_location)[1]

            if is_on_wall(ball_location, True):
                if is_on_wall(agent.me.location, False):
                    distance = car_to_ball.magnitude()
                    direction = car_to_ball.normalize()
                else:
                    distance = ((ball_location.flatten() + towards_wall * ball_location[2]) - agent.me.location + Vector3(0, 0, 300)).magnitude()
                    direction = ((ball_location.flatten() + towards_wall * ball_location[2]) - agent.me.location + Vector3(0, 0, 300)).normalize()
            else:
                distance = car_to_ball.flatten().magnitude()
                direction = car_to_ball.flatten().normalize()

            estimated_time, forwards = eta(agent.me, ball_location, direction, distance)

            direction = car_to_ball.normalize()

            if estimated_time < time_remaining:
                for pair in targets:
                    #First we correct the target coordinates to account for the ball's radius
                    #If swapped == True, the shot isn't possible because the ball wouldn't fit between the targets
                    left, right, swapped = post_correction(ball_location, targets[pair][0], targets[pair][1])
                    if not swapped:
                        #Now we find the easiest direction to hit the ball in order to land it between the target points
                        left_vector = (left - ball_location).normalize()
                        right_vector = (right - ball_location).normalize()
                        best_shot_vector = direction.clamp(left_vector, right_vector)
                    
                        #Check to make sure our approach is inside the field
                        if in_field(ball_location - (200*best_shot_vector), 1):
                            #The slope represents how close the car is to the chosen vector, higher = better
                            #A slope of 1.0 would mean the car is 45 degrees off
                            slope = find_slope(best_shot_vector, car_to_ball)
                            if forwards:
                                if ball_location[2] < 300:
                                    hits[pair].append(shoot(ball_location, intercept_time, best_shot_vector, 1))
                                if ball_location[2] < 500:
                                    hits[pair].append(double_jump(ball_location, intercept_time, best_shot_vector, 1))
                                elif is_on_wall(ball_location, True):
                                    hits[pair].append(wall_hit(ball_location, intercept_time, best_shot_vector, 1))
                            else:
                                if ball_location[2] <= 300:
                                    hits[pair].append(shoot(ball_location, intercept_time, best_shot_vector, -1))
        else:
            i += 1
    return hits