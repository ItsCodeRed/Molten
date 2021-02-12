from util.objects import *
from util.utils import *
from util.mechanics import *
from util.tools import *

class Molten(MoltenAgent):

    def solo_strat(agent):
        ball_location = agent.ball.location
        ball_to_me = ball_location - agent.me.location
        my_goal_location = agent.friend_goal.location

        if len(agent.foes) > 0:
            closest_foe = agent.foes[0]
            for foe in agent.foes:
                if foe.location.distance(ball_location) < closest_foe.location.distance(ball_location):
                    closest_foe = foe

            my_eta = eta(agent.me, ball_location, (ball_location - agent.me.location).normalize(), agent.me.location.distance(ball_location))
            foe_eta = eta(closest_foe, ball_location, (ball_location - closest_foe.location).normalize(), closest_foe.location.distance(ball_location))

            enemy_approaching = closest_foe.velocity.dot(ball_location - closest_foe.location) > 0

            shot_incoming = (my_goal_location - (ball_location + closest_foe.velocity + agent.ball.velocity)).magnitude() < 4000

            shadow_pos = (my_goal_location + ball_location + closest_foe.velocity).flatten() / 2
            shadow_pos.x = cap(shadow_pos.x - sign(shadow_pos.x) * 1000, -3500, 3500)
            shadow_pos.y = cap(shadow_pos.y, -4500, 4500)

            if agent.kickoff and len(agent.stack) < 1:
                agent.push(kickoff(agent.me.location.x))
            elif my_eta - 0.2 > foe_eta and not shot_incoming and enemy_approaching:
                if len(agent.stack) < 1:
                    agent.push(goto(shadow_pos, ball_to_me, 2300))
                elif not isinstance(agent.stack[-1], goto) and not isinstance(agent.stack[-1], flip) and not isinstance(agent.stack[-1], recovery):
                    agent.pop()
                elif isinstance(agent.stack[-1], goto):
                    agent.stack[-1].update(shadow_pos, ball_to_me, 2300)
            elif shot_incoming:
                save(agent)
            else:
                attack(agent)
        else:
            if agent.kickoff and len(agent.stack) < 1:
                agent.push(kickoff(agent.me.location.x))
            else:
                attack(agent)


    def duo_strat(agent):
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
            (agent.kickoff and ((abs(my_eta - friend_eta) < 0.1 and sign(agent.me.location.x) != side(agent.team)) or (my_distance < friend_distance))) or my_distance < 500 and me_back:
            agent.rotation_index = 0
        else:
            agent.rotation_index = 1

        foe_goal_distance_to_foe_one = (agent.foe_goal.location - agent.foes[0].location).magnitude()
        foe_goal_distance_to_foe_two = (agent.foe_goal.location - agent.foes[1].location).magnitude()
        foe_goal_distance_to_ball = (agent.foe_goal.location - agent.ball.location).magnitude()

        foe_one_back = foe_goal_distance_to_foe_one < foe_goal_distance_to_ball
        foe_two_back = foe_goal_distance_to_foe_two < foe_goal_distance_to_ball

        closest_foe = agent.foes[0]
        for foe in agent.foes:
            if foe.location.distance(ball_location) < closest_foe.location.distance(ball_location):
                closest_foe = foe

        shadow_pos = (my_goal_location + ball_location + closest_foe.velocity).flatten() / 2
        shadow_pos.x = cap(shadow_pos.x - sign(shadow_pos.x) * 1000, -3500, 3500)
        shadow_pos.y = cap(shadow_pos.y, -4500, 4500)
        distance_to_shadow = (shadow_pos - agent.me.location).flatten().magnitude()

        if agent.rotation_index == 0:
            if agent.kickoff:
                if len(agent.stack) < 1:
                    agent.push(kickoff(agent.me.location.x))
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
                elif not isinstance(agent.stack[-1], goto) and not isinstance(agent.stack[-1], flip) and not isinstance(agent.stack[-1], recovery):
                    agent.pop()
                elif isinstance(agent.stack[-1], goto):
                    agent.stack[-1].update(shadow_pos, ball_to_me, 2300)
            else:
                if len(agent.stack) < 1:
                    agent.push(goto(shadow_pos, ball_to_me, 1400))
                elif not isinstance(agent.stack[-1], goto) and not isinstance(agent.stack[-1], flip) and not isinstance(agent.stack[-1], recovery):
                    agent.pop()
                elif isinstance(agent.stack[-1], goto):
                    agent.stack[-1].update(shadow_pos, ball_to_me, 1400)

    def team_strat(agent):
        ball_location = agent.ball.location
        my_goal_location = agent.friend_goal.location

        ball_to_me = ball_location - agent.me.location

        if agent.time > agent.update_time or agent.latest_touched_time != agent.ball.latest_touched_time:
            cars = agent.friends.copy()
            cars.extend(agent.foes)
            cars.append(agent.me)
            focus = find_next_hit(agent, cars)
            agent.first_pos = focus if focus != None else ball_location
            agent.latest_touched_time = agent.ball.latest_touched_time
            agent.update_time = agent.time + 0.1

        closest_foe = agent.foes[0]
        for foe in agent.foes:
            if (eta(foe, agent.first_pos) < eta(closest_foe, agent.first_pos) and is_back(agent, foe) == is_back(agent, closest_foe)) or (is_back(agent, foe) and not is_back(agent, closest_foe)) and not foe.demolished:
                closest_foe = foe

        third_pos = my_goal_location.flatten()
        third_pos.x = third_pos.x - 700 * sign(agent.first_pos.x)
        third_pos.y = third_pos.y - 600 * sign(third_pos.y)

        second_pos = (my_goal_location + agent.first_pos * 3).flatten() / 4
        second_pos.x = cap(second_pos.x / 2, -3500, 3500)
        second_pos.y = cap(second_pos.y, -4000, 4000)

        first_mate = None
        friends_back = 0
        for friend in agent.friends:
            if first_mate == None or ((eta(friend, agent.first_pos) < eta(first_mate, agent.first_pos) and is_back(agent, friend) == is_back(agent, first_mate)) or (is_back(agent, friend) and not is_back(agent, first_mate)) and not friend.demolished):
                first_mate = friend
            if is_back(agent, friend):
                friends_back += 1
        
        second_mate = None
        for friend in agent.friends:
            if second_mate == None or ((eta(friend, second_pos) < eta(second_mate, second_pos) and is_back(agent, friend) == is_back(agent, second_mate)) or (is_back(agent, friend) and not is_back(agent, second_mate)) and not friend.demolished):
                second_mate = friend

        third_mate = None
        for friend in agent.friends:
            if third_mate == None or ((eta(friend, third_pos) < eta(third_mate, third_pos) and is_back(agent, friend) == is_back(agent, third_mate)) or (is_back(agent, friend) and not is_back(agent, third_mate)) and not friend.demolished):
                third_mate = friend

        if agent.kickoff:
            agent.rotation_index = 0
            for friend in agent.friends:
                if (eta(friend, ball_location) < eta(agent.me, ball_location) and (abs(eta(friend, ball_location) - eta(agent.me, ball_location)) > 0.1)) or (abs(eta(friend, ball_location) - eta(agent.me, ball_location)) < 0.1 and sign(agent.me.location.x) == side(agent.team)):
                    agent.rotation_index += 1

        if agent.rotation_index == 0:
            if agent.kickoff and not agent.me.airborne and len(agent.stack) < 1:
                agent.push(kickoff(agent.me.location.x))
            elif ((eta(agent.me, agent.first_pos) > eta(closest_foe, agent.first_pos) and sign(agent.first_pos.y) != side(agent.team) and is_back(agent, agent.me) and \
                eta(agent.me, agent.first_pos) > eta(first_mate, agent.first_pos) and not first_mate.airborne) or (eta(agent.me, agent.first_pos) > eta(first_mate, agent.first_pos) and agent.me.boost < 40 and not first_mate.airborne) or not is_back(agent, agent.me)) and len(agent.stack) < 1 and not agent.me.airborne :
                agent.rotation_index = 2
            elif eta(agent.me, agent.first_pos) > eta(first_mate, agent.first_pos) and not first_mate.airborne and agent.me.boost > 40 and len(agent.stack) < 1 and not agent.me.airborne:
                agent.rotation_index = 1
            elif sign(agent.first_pos.y) == side(agent.team) and is_back(agent, closest_foe):
                save(agent)
            else:
                attack(agent)
        elif agent.rotation_index == 1:
            if agent.me.airborne and len(agent.stack) < 1:
                agent.push(recovery())
            elif len(agent.stack) < 1:
                agent.push(goto(second_pos, ball_to_me, not is_back(agent, agent.me), (5200 - side(agent.team) * agent.first_pos.y) / 5200))
            elif (eta(agent.me, agent.first_pos) < eta(first_mate, agent.first_pos) or (eta(agent.me, agent.first_pos) < eta(first_mate, agent.first_pos) + 0.5 and first_mate.airborne) or not is_back(agent, first_mate)) and not agent.kickoff:
                agent.rotation_index = 0
            elif ((eta(agent.me, third_pos) > eta(agent.me, second_pos) > eta(second_mate, second_pos) and not second_mate.airborne and agent.me.boost < second_mate.boost) or eta(agent.me, second_pos) < eta(agent.me, third_pos) < eta(third_mate, my_goal_location.flatten())) and not agent.kickoff:
                agent.rotation_index = 2
            elif len(agent.stack) > 0 and not isinstance(agent.stack[-1], goto) and not isinstance(agent.stack[-1], flip) and not isinstance(agent.stack[-1], recovery):
                agent.pop()
            elif isinstance(agent.stack[-1], goto):
                agent.stack[-1].update(second_pos, ball_to_me, not is_back(agent, agent.me), (5200 - side(agent.team) * agent.first_pos.y) / 5200)
        else:
            if agent.me.airborne and len(agent.stack) < 1:
                agent.push(recovery())
            elif eta(third_mate, my_goal_location.flatten()) < agent.first_pos.distance(third_pos) / 2500 and agent.me.location.distance(third_pos) < 1000 and not agent.kickoff:
                agent.rotation_index = 1
            elif eta(agent.me, agent.first_pos) < eta(first_mate, agent.first_pos) and is_back(agent, agent.me) and not agent.kickoff:
                agent.rotation_index = 0
            elif len(agent.stack) > 0 and agent.me.location.distance(third_pos) < 200:
                agent.pop()
            elif len(agent.stack) < 1 and agent.me.location.distance(third_pos) > 200:
                agent.push(goto(third_pos, ball_to_me, not is_back(agent, agent.me), (5200 - side(agent.team) * agent.first_pos.y) / 5200))
            elif len(agent.stack) > 0 and not isinstance(agent.stack[-1], goto) and not isinstance(agent.stack[-1], flip) and not isinstance(agent.stack[-1], recovery):
                agent.pop()
            elif len(agent.stack) > 0 and isinstance(agent.stack[-1], goto):
                agent.stack[-1].update(third_pos, ball_to_me, not is_back(agent, agent.me), (5200 - side(agent.team) * agent.first_pos.y) / 5200)
    
    def atba_strat(agent):
        if len(agent.stack) < 1:
            if agent.kickoff:
                agent.push(kickoff(agent.me.location.x))
            else:
                attack(agent)
    
    def run(agent):
        # find_fastest_hits(agent, np.append(np.append(agent.friends, agent.foes), agent.me))
        agent.debug_stack()
        agent.me.hitbox.render(agent, [255, 255, 255])
        
        if len(agent.friends) == 0:
            Molten.solo_strat(agent)
        elif len(agent.friends) == 1:
            Molten.duo_strat(agent)
        elif len(agent.friends) > 1:
            Molten.team_strat(agent)
        else:
            Molten.atba_strat(agent)

    def is_hot_reload_enabled(self):
        return False