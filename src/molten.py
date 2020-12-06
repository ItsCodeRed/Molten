from util.objects import *
from util.utils import *
from util.mechanics import *
from util.tools import *

class Molten(MoltenAgent):

    def onevsone_strats(agent):
        ball_location = agent.ball.location

        ball_to_me = ball_location - agent.me.location
        my_distance = ball_to_me.magnitude()

        my_goal_location = agent.friend_goal.location
        my_goal_distance_to_me = (my_goal_location - agent.me.location).magnitude()
        my_goal_distance_to_ball = (my_goal_location - ball_location).magnitude()

        foe_goal_location = agent.foe_goal.location
        foe_goal_distance_to_ball = (foe_goal_location - ball_location).magnitude()

        ball_to_foe = ball_location - agent.foes[0].location
        foe_distance = ball_to_foe.magnitude()

        my_eta = eta(agent.me, ball_location, ball_to_me.normalize(), my_distance)
        foe_eta = eta(agent.foes[0], ball_location, ball_to_foe.normalize(), foe_distance)

        shot_incoming = (my_goal_location - (ball_location + agent.foes[0].velocity + agent.ball.velocity)).magnitude() < 4000

        shadow_pos = (my_goal_location + ball_location + agent.foes[0].velocity).flatten() / 2
        distance_to_shadow = (shadow_pos - agent.me.location).flatten().magnitude()

        upfield_left = Vector3(-side(agent.index) * 4096, ball_location.y - side(agent.index) * 1000, 0)
        upfield_right = Vector3(side(agent.index) * 4096, ball_location.y - side(agent.index) * 1000, 0)

        if agent.kickoff and len(agent.stack) < 1:
            agent.push(kickoff(sign(agent.me.location.x + 0.01)))
        elif shot_incoming:
            save(agent)
        else:
            attack(agent)

    def twovstwo_strats(agent):
        ball_location = agent.ball.location

        ball_to_me = ball_location - agent.me.location
        my_distance = ball_to_me.magnitude()

        ball_to_friend = ball_location - agent.friends[0].location
        friend_distance = ball_to_friend.magnitude()

        my_eta = eta(agent.me, ball_location, ball_to_me.normalize(), my_distance)
        friend_eta = eta(agent.friends[0], ball_location, ball_to_friend.normalize(), friend_distance)

        my_goal_location = agent.friend_goal.location
        my_goal_distance_to_me = (my_goal_location - agent.me.location).magnitude()
        my_goal_distance_to_friend = (my_goal_location - agent.friends[0].location).magnitude()
        my_goal_distance_to_ball = (my_goal_location - ball_location).magnitude()

        me_back = my_goal_distance_to_me < my_goal_distance_to_ball
        friend_back = my_goal_distance_to_friend < my_goal_distance_to_ball

        if (not agent.kickoff and ((my_eta < friend_eta and me_back == friend_back) or (me_back and not friend_back))) or \
            (agent.kickoff and ((abs(my_eta - friend_eta) < 0.1 and sign(agent.me.location.x) == side(agent.team)) or (my_distance < friend_distance))):
            agent.rotation_index = 0
        else:
            agent.rotation_index = 1

        ball_to_foe_one = ball_location - agent.foes[0].location
        ball_to_foe_two = ball_location - agent.foes[1].location

        foe_one_eta = eta(agent.foes[0], ball_location, ball_to_foe_one.normalize(), ball_to_foe_one.magnitude())
        foe_two_eta = eta(agent.foes[1], ball_location, ball_to_foe_two.normalize(), ball_to_foe_two.magnitude())

        foe_goal_distance_to_foe_one = (agent.foe_goal.location - agent.foes[0].location).magnitude()
        foe_goal_distance_to_foe_two = (agent.foe_goal.location - agent.foes[1].location).magnitude()
        foe_goal_distance_to_ball = (agent.foe_goal.location - agent.ball.location).magnitude()

        foe_one_back = foe_goal_distance_to_foe_one < foe_goal_distance_to_ball
        foe_two_back = foe_goal_distance_to_foe_two < foe_goal_distance_to_ball

        if (foe_one_eta < foe_two_eta and foe_one_back == foe_two_back) or (foe_one_back and not foe_two_back):
            closest_foe = agent.foes[0]
        else:
            closest_foe = agent.foes[1]

        shadow_pos = (my_goal_location + ball_location + closest_foe.velocity).flatten() / 2
        shadow_pos.x = -shadow_pos.x / 2
        distance_to_shadow = (shadow_pos - agent.me.location).flatten().magnitude()

        if agent.rotation_index == 0:
            if agent.kickoff:
                if len(agent.stack) < 1:
                    agent.push(kickoff(sign(agent.me.location.x + 0.01)))
                elif isinstance(agent.stack[-1], goto):
                    agent.pop()
            elif sign(ball_location.y) == side(agent.team) and (foe_one_back or foe_two_back):
                save(agent)
            else:
                attack(agent)
        else:
            if (my_goal_distance_to_friend + 500 > my_goal_distance_to_ball < my_goal_distance_to_me) or (my_eta < friend_eta):
                if len(agent.stack) < 1:
                    agent.push(goto(shadow_pos, ball_to_me, 2300))
                elif not isinstance(agent.stack[-1], goto):
                    agent.pop()
                else:
                    agent.stack[-1].update(shadow_pos, ball_to_me, 2300)
            else:
                if len(agent.stack) < 1:
                    agent.push(goto(shadow_pos, ball_to_me, 1400))
                elif not isinstance(agent.stack[-1], goto):
                    agent.pop()
                else:
                    agent.stack[-1].update(shadow_pos, ball_to_me, 1400)
    
    def atba_strats(agent):
        if len(agent.stack) < 1:
            if agent.kickoff:
                agent.push(kickoff(sign(agent.me.location.x + 0.01)))
            else:
                upfield_left = Vector3(-side(agent.index) * 4096, agent.ball.location.y - side(agent.index) * 1000, 0)
                upfield_right = Vector3(side(agent.index) * 4096, agent.ball.location.y - side(agent.index) * 1000, 0)
                targets = {"goal":(agent.foe_goal.left_post, agent.foe_goal.right_post), "upfield":(upfield_left, upfield_right)}
                shots = find_hits(agent, targets)
                if len(shots["goal"]) > 0:
                    agent.push(shots["goal"][0])
                elif len(shots["upfield"]) > 0:
                    agent.push(shots["upfield"][0])
                else:
                    agent.push(short_shot(agent.foe_goal.location))
    
    def run(agent):
        agent.debug_stack()
        agent.me.hitbox.render(agent, [255, 255, 255])

        is_twovstwo = len(agent.foes) == len(agent.friends) + 1 == 2
        is_onevsone = len(agent.foes) == len(agent.friends) + 1 == 1
        if is_onevsone:
            Molten.onevsone_strats(agent)
        elif is_twovstwo:
            Molten.twovstwo_strats(agent)
        else:
            Molten.atba_strats(agent)

