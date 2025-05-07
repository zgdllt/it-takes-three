import logging
import math

from agent.agent import Agent
import heapq


async def selectBuff(agent: Agent):
    # Your code here.
    # Here is an example of how to select a buff.
    # Always select the first buff in the available buff list.
    available_buffs = agent.availiable_buffs
    if available_buffs is None:
        logging.warning("No available buffs.")
        return
    await agent.select_buff(available_buffs.buffs[0].name)

def judge_bullet(bullet, player, walls):
    bullet_pos = bullet.position
    bullet_angle = bullet_pos.angle
    player_pos = player.position
    bullet_speed = bullet.speed
    distance = 0
    for i in range(1, 20):
        distance += bullet_speed
        bullet_pos.x += bullet_speed * math.cos(bullet_angle)
        bullet_pos.y += bullet_speed * math.sin(bullet_angle)
        for wall in walls:
            if wall.position.angle == 0:
                # For horizontal walls (angle == 0)
                wall_y = wall.position.y
                wall_x = wall.position.x
                wall_half_length = 10  # Assuming default wall length
                collision_distance = 0.5  # Collision threshold

                # Check if bullet is close enough to the wall to be considered a collision
                if abs(bullet_pos.y - wall_y) < collision_distance and abs(bullet_pos.x - wall_x) < wall_half_length:
                    bullet_angle = 360 - bullet_angle
            if wall.position.angle == 90:
                # For vertical walls (angle == 90)
                wall_x = wall.position.x
                wall_y = wall.position.y
                wall_half_length = 10
                collision_distance = 0.5
                # Check if bullet is close enough to the wall to be considered a collision
                if abs(bullet_pos.x - wall_x) < collision_distance and abs(bullet_pos.y - wall_y) < wall_half_length:
                    bullet_angle = 540 - bullet_angle
                    if bullet_angle > 360:
                        bullet_angle -= 360
        dist = math.hypot(player_pos.x - bullet_pos.x, player_pos.y - bullet_pos.y)
        if dist < 0.6:
            is_safe = False
            break
    return is_safe, bullet_angle, distance
def find_rival(walls,self_info, opponent_info):
    """Find the shortest path to the rival player using A* algorithm."""
    start = (int(self_info.position.x), int(self_info.position.y))
    goal = (int(opponent_info.position.x), int(opponent_info.position.y))
    # A* algorithm
    open_set = {}
    came_from = {}
    g_score = {start: 0}
    f_score = {start: math.sqrt((start[0] - goal[0])**2 + (start[1] - goal[1])**2)}
    
    open_heap = [(f_score[start], start)]
    heapq.heapify(open_heap)
    
    directions = [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (-1, 1), (1, -1), (-1, -1)]
    
    while open_heap:
        _, current = heapq.heappop(open_heap)
        
        if current == goal:
            # Reconstruct path
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            path.reverse()
            return path
        
        for dx, dy in directions:
            next_x, next_y = current[0] + dx, current[1] + dy
            neighbor = (next_x, next_y)
            
            # Check if move crosses any wall
            wall_crossed = False
            for wall in walls:
                wx, wy = wall.position.x, wall.position.y
                if wall.position.angle == 0:  # Horizontal wall
                    # Wall from (wx-10, wy) to (wx, wy)
                    if min(current[0], next_x) <= wx and max(current[0], next_x) >= wx-10 and (
                        (current[1] <= wy and next_y >= wy) or (current[1] >= wy and next_y <= wy)):
                        wall_crossed = True
                        break
                elif wall.position.angle == 90:  # Vertical wall
                    # Wall from (wx, wy-10) to (wx, wy)
                    if min(current[1], next_y) <= wy and max(current[1], next_y) >= wy-10 and (
                        (current[0] <= wx and next_x >= wx) or (current[0] >= wx and next_x <= wx)):
                        wall_crossed = True
                        break
            
            if wall_crossed:
                continue
            
            tentative_g_score = g_score[current] + math.sqrt(dx*dx + dy*dy)
            
            if tentative_g_score < g_score.get(neighbor, float('inf')):
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + math.sqrt((neighbor[0] - goal[0])**2 + (neighbor[1] - goal[1])**2)
                
                if neighbor not in open_set:
                    open_set[neighbor] = True
                    heapq.heappush(open_heap, (f_score[neighbor], neighbor))
    return None  # No path found
def target_rival(walls,self_info, opponent_info):
    for i in range(0, 359):
        bullet_pos = self_info.position
        bullet_pos.x += 0.8 * math.cos(i)
        bullet_pos.y += 0.8 * math.sin(i)
        bullet_pos.angle = i
        is_safe, direction, distance = judge_bullet(bullet_pos, opponent_info, walls)
        if not is_safe:
            return direction
    return 0
async def loop(agent: Agent):
    # Your code here.
    # Here is an example of how to use the agent.
    # Select a buff if available
    await selectBuff(agent)
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

    environment_info = agent.environment_info
    assert environment_info is not None
    walls = environment_info.walls
    bullets = environment_info.bullets

    px = self_info.position.x
    py = self_info.position.y
    player_angle = self_info.position.angle

    is_safe = True

    wall_safe_distance = 1.0
    for wall in walls:
        wall_pos = wall.position
        wx = wall_pos.x
        wy = wall_pos.y
        wall_angle = wall_pos.angle

        if wall_angle == 0:
            distance = wy - py
            if abs(distance) < wall_safe_distance:
                is_safe = False
                if math.tan(player_angle) > 0:
                    await agent.turn_clockwise()
                else:
                    await agent.turn_counter_clockwise()

        elif wall_angle == 90:
            distance = wx - px
            if abs(distance) < wall_safe_distance:
                is_safe = False
                if math.tan(player_angle) > 0:
                    await agent.turn_counter_clockwise()
                else:
                    await agent.turn_clockwise()
        else:
            continue

    bullet_danger_distance = 3.0
    for bullet in bullets:
        is_safe, direction, distace = judge_bullet(bullet, self_info, walls)
        if not is_safe:
            perp_direction = (direction + 90) % 360
            diff = ((perp_direction - player_angle) % 360)
            if diff <= 180:
                if diff > 45:
                    await agent.turn_counter_clockwise(diff//45)
                else:
                    await agent.turn_counter_clockwise(1)
            else:
                diff = (360 - diff) % 360
                if diff > 45:
                    await agent.turn_clockwise(diff//45)
                else:
                    await agent.turn_clockwise(1)
            await agent.turn_clockwise(0)
            await agent.move_forward()
            break

    if is_safe and self_info.weapon.current_bullets > 0:
        distance = math.hypot(opponent_info.position.x - px, opponent_info.position.y - py)
        if distance > 20:
            path= find_rival(walls, self_info, opponent_info)
            if path is not None:
                for point in path:
                    target_x, target_y = point
                    if abs(target_x - px) > 0.5 or abs(target_y - py) > 0.5:
                        if target_x > px:
                            await agent.move_forward()
                        elif target_x < px:
                            await agent.move_backward()
                        if target_y > py:
                            await agent.turn_clockwise(1)
                        elif target_y < py:
                            await agent.turn_counter_clockwise(1)
            else:
                logging.warning("No path found to the rival.")
        else:
            target = target_rival(walls, self_info, opponent_info)
            diff = ((target - player_angle) % 360)
            while diff > 45:
                if diff <= 180:
                    if diff > 45:
                        await agent.turn_counter_clockwise(diff // 45)
                    else:
                        await agent.turn_counter_clockwise(1)
                else:
                    diff = (360 - diff) % 360
                    if diff > 45:
                        await agent.turn_clockwise(diff // 45)
                    else:
                        await agent.turn_clockwise(1)
                    diff = ((target - player_angle) % 360)
            await agent.turn_clockwise(0)
            await agent.attack()

    for skill in self_info.skills:
        if skill.current_cooldown == 0:
            await agent.use_skill(skill.name)
            break