from utils import *

class atba():
    #An example routine that just drives towards the ball at max speed
    def run(self, agent):
        relative_target = agent.ball.location - agent.me.location
        local_target = agent.me.local(relative_target)
        default_orient(agent, local_target)
        default_throttle(agent, 2300)

class pop():

    def __init__(self, ball_location, intercept_time):
        self.ball_location = ball_location
        self.intercept_time = intercept_time
    
    def run(self, agent):
        raw_time_remaining = self.intercept_time - agent.time
        time_remaining = cap(raw_time_remaining,0.001,10.0)

        relative_target = self.ball_location - agent.me.location
        local_target = agent.me.local(relative_target)
        distance = relative_target.magnitude()
        speed_required = distance / time_remaining

        default_orient(agent, local_target)
        default_throttle(agent, speed_required)