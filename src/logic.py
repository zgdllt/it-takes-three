import logging
import math
import pygame
from agent.agent import Agent
class Strategy:
    async def update(self, agent: Agent):
        raise NotImplementedError
    
class DodgeStrategy(Strategy):
    def __init__(self, angle):
        self.angle = angle
        self.done = False

    async def update(self, agent: Agent):
        if self.done:
            return False
        # rotate towards safe direction
        await agent.turn_clockwise(self.angle)
        # move forward to dodge
        await agent.move_forward()
        self.done = True
        return False

class ContactStrategy(Strategy):
    def __init__(self, path):
        self.path = path or []
        self.index = 0

    async def update(self, agent: Agent):
        if self.index >= len(self.path):
            return False
        x, y = self.path[self.index]
        info = agent.all_player_info
        self_info = next(p for p in info if p.token == agent.token)
        px, py = self_info.position.x, self_info.position.y
        # simple move logic
        if abs(x - px) > abs(y - py):
            if x > px:
                await agent.move_forward()
            else:
                await agent.move_backward()
        else:
            if y > py:
                await agent.turn_clockwise()
            else:
                await agent.turn_counter_clockwise()
        self.index += 1
        return True

class AttackStrategy(Strategy):
    def __init__(self):
        self.done = False

    async def update(self, agent: Agent):
        if self.done:
            return False
        info = agent.all_player_info
        self_info = next(p for p in info if p.token == agent.token)
        op = next(p for p in info if p.token != agent.token)
        # aim
        angle = target_rival(agent.environment_info.walls, self_info, op)
        await agent.turn_clockwise(angle)
        await agent.attack()
        self.done = True
        return False

# current strategy instance
current_strategy = None

async def selectBuff(agent: Agent):
    # Your code here.
    # Here is an example of how to select a buff.
    # Always select the first buff in the available buff list.
    available_buffs = agent.availiable_buffs
    assert available_buffs is not None

    await agent.select_buff(available_buffs.buffs[0].name)

async def get_player_info(agent: Agent):
    player_info_list = agent.all_player_info
    assert player_info_list is not None

    self_info = None
    opponent_info = None

    for player in player_info_list:
        if player.token == agent.token:
            self_info = player
        else:
            opponent_info = player
    logging.debug(f"Self: {self_info.position}, Opponent: {opponent_info.position}")
    return self_info, opponent_info

def judge_bullet(bullet_pos, bullet_speed, player, walls):
    bullet_angle = bullet_pos.angle
    player_pos = player.position
    is_safe = True
    distance = 0
    for i in range(1, 100):
        distance += bullet_speed
        bullet_pos.x += bullet_speed * math.cos(bullet_angle)
        bullet_pos.y += bullet_speed * math.sin(bullet_angle)
        for wall in walls:
            wall_y = wall.position.y*10
            wall_x = wall.position.x*10
            wall_length = 10  # Assuming default wall length
            collision_distance = 1  # Collision threshold
            if wall.position.angle == 0:
                # For horizontal walls (angle == 0)
                # Check if bullet is close enough to the wall to be considered a collision
                if abs(bullet_pos.y - wall_y) < collision_distance and 0 < bullet_pos.x - wall_x < wall_length:
                    bullet_angle = 180 - bullet_angle
            if wall.position.angle == 90:
                # For vertical walls (angle == 90)
                # Check if bullet is close enough to the wall to be considered a collision
                if abs(bullet_pos.x - wall_x) < collision_distance and 0 < bullet_pos.y - wall_y < wall_length:
                    bullet_angle = - bullet_angle
            bullet_angle = bullet_angle % 360
        dist = math.hypot(player_pos.x - bullet_pos.x, player_pos.y - bullet_pos.y)
        if dist < 1:
            is_safe = False
            break
    return is_safe, bullet_angle, distance

def find_rival(walls, self_info, opponent_info):
    # Your code here.
    pass

def target_rival(walls,self_info, opponent_info):
    for i in range(0, 359):
        bullet_pos = self_info.position
        bullet_pos.x += 0.8 * math.cos(i/180*math.pi)
        bullet_pos.y += 0.8 * math.sin(i/180*math.pi)
        bullet_pos.angle = i
        is_safe, direction, distance = judge_bullet(bullet_pos, bulletspeed, opponent_info, walls)
        if not is_safe:
            return i
    return 360

async def loop(agent: Agent):
    global current_strategy
    # fetch player info and environment
    self_info, opponent_info = await get_player_info(agent)
    env = agent.environment_info
    if env is None:
        return
    walls = env.walls
    bullets = env.bullets

    # continue current strategy if active
    if current_strategy:
        active = await current_strategy.update(agent)
        if active:
            return
        current_strategy = None

    # check bullet threats
    for bullet in bullets:
        is_safe, direction, _ = judge_bullet(bullet.position, bullet.speed, self_info, walls)
        if not is_safe:
            current_strategy = DodgeStrategy(direction)
            return

    # compute distance to opponent
    dx = opponent_info.position.x - self_info.position.x
    dy = opponent_info.position.y - self_info.position.y
    distance = math.hypot(dx, dy)
    ATTACK_THRESHOLD = 20
    if distance > ATTACK_THRESHOLD:
        path = find_rival(walls, self_info, opponent_info)
        current_strategy = ContactStrategy(path)
        return

    # default attack strategy
    current_strategy = AttackStrategy()
    await current_strategy.update(agent)