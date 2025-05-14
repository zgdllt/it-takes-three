import logging
import math
import pygame
from agent.agent import Agent


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
    # Your code here.
    # Here is an example of how to use the agent.
    # Select a buff if available
    global state
    global bulletspeed
    bulletspeed=2
    await selectBuff(agent)
    assert agent.all_player_info is not None
    self_info, opponent_info = await get_player_info(agent)
    logging.debug(f"Self: {self_info.position}, Opponent: {opponent_info.position}")

    environment_info = agent.environment_info
    assert environment_info is not None
    walls = environment_info.walls
    bullets = environment_info.bullets

    px = self_info.position.x
    py = self_info.position.y
    player_angle = self_info.position.angle
    is_safe = True
    # anti-rotation
    agent.turn_clockwise(0)
    bullet_danger_distance = 5.0
    for bullet in bullets:
        state="avoiding"
        is_safe, direction, distace = judge_bullet(bullet.position,bullet.speed, self_info, walls)
        bulletspeed = bullet.speed
        if not is_safe:
            perp_direction = (direction + 90) % 360
            diff = (perp_direction - int(player_angle*180/math.pi)) % 360
            if diff <= 180:
                if diff > 45:
                    await agent.turn_counter_clockwise(45)
                else:
                    await agent.turn_counter_clockwise(diff)
            else:
                diff = (360 - diff) % 360
                if diff > 45:
                    await agent.turn_clockwise(45)
                else:
                    await agent.turn_clockwise(diff)
            await agent.turn_clockwise(0)
            await agent.move_forward()
            break

    wall_safe_distance = 5.0
    if is_safe and self_info.weapon.current_bullets > 0:
        distance = math.hypot(opponent_info.position.x - px, opponent_info.position.y - py)
        state="moving"
        await agent.move_forward()
        next_pos = (px + wall_safe_distance * math.cos(player_angle), py + wall_safe_distance * math.sin(player_angle))
        new_info,_= await get_player_info(agent)
        new_pos = (new_info.position.x, new_info.position.y)
        if new_pos == (px, py):
            await agent.move_backward()
            await agent.turn_clockwise()
        for wall in walls:
            wall_pos = wall.position
            wx = wall_pos.x * 10
            wy = wall_pos.y * 10
            nx = next_pos[0]
            ny = next_pos[1]
            wall_angle = wall_pos.angle
            if wall_angle == 0:
                intersection_x=(wy-py)/math.tan(player_angle)+px
                if (py < wy and ny > wy) or (py > wy and ny < wy) and 0 < intersection_x - nx < 10:
                    await agent.move_backward()
                    if math.tan(player_angle) > 0:
                        await agent.turn_clockwise()
                    else:
                        await agent.turn_counter_clockwise()
                break
            elif wall_angle == 90:
                intersection_y=(wx-px)*math.tan(player_angle)+py
                if (px < wx and nx > wx) or (px > wx and nx < wx) and 0 < intersection_y - ny < 10:
                    await agent.move_backward()
                    if math.tan(player_angle) > 0:
                        await agent.turn_counter_clockwise()
                    else:
                        await agent.turn_clockwise()
                break
        target = target_rival(walls, self_info, opponent_info)
        if target != 360: 
            await agent.move_forward(0)
            diff = (target - int(player_angle*180/math.pi)) % 360
            while abs(diff) > 10:
                if diff < 0:
                    if diff < -45:
                        await agent.turn_clockwise()
                    else:
                        await agent.turn_clockwise(-diff)
                else:
                    if diff > 45:
                        await agent.turn_counter_clockwise()
                    else:
                        await agent.turn_counter_clockwise(diff)
                self_info, opponent_info = await get_player_info(agent)
                player_angle = self_info.position.angle
                diff = target - int(player_angle * 180 / math.pi)
                diff = diff % 360
            await agent.turn_clockwise(0)
            for _ in range(0, 10):
                await agent.attack()
    for skill in self_info.skills:
        if skill.current_cooldown == 0:
            await agent.use_skill(skill.name)
            break