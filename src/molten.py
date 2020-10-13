from objects import *
from utils import *
from mechanics import *
from tools import *

class Molten(MoltenAgent):
    
    def run(agent):
        
        if len(agent.stack) < 1:
            targets = {"goal":(agent.foe_goal.left_post, agent.foe_goal.right_post)}
            shots = find_hits(agent, targets)
            if len(shots["goal"]) > 0:
                agent.push(shots["goal"][0])
            else:
                agent.push(atba())


