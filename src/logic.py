import logging
import math

from agent.agent import Agent


async def selectBuff(agent: Agent):
    # Your code here.
    # Here is an example of how to select a buff.
    # Always select the first buff in the available buff list.
    available_buffs = agent.availiable_buffs
    assert available_buffs is not None

    await agent.select_buff(available_buffs.buffs[0].name)

# class BallisticSegment:
#     def __init__(self, shell_id, seq, start, end, angle, length, distance_to_target):
#         self.shell_id = shell_id
#         self.seq = seq
#         self.start = start  # Position object with x, y coordinates
#         self.end = end      # Position object with x, y coordinates
#         self.angle = angle
#         self.length = length
#         self.distance_to_target = distance_to_target
#         self.center = {'x': (start.x + end.x) / 2, 'y': (start.y + end.y) / 2}
    
#     @staticmethod
#     def invalid():
#         return BallisticSegment(-1, -1, None, None, 0, 0, 0)
    
#     def is_valid(self):
#         return self.shell_id != -1


# def calculate_bullet_trajectories(bullets, walls, target_position):
#     bullet_segments = []
#     for bullet in bullets:
#         bullet_pos = bullet.position
#         bullet_angle = bullet_pos.angle
#         bullet_distance_to_target = math.hypot(bullet_pos.x - target_position.x, bullet_pos.y - target_position.y)

#         # Calculate the start and end points of the bullet trajectory
#         start_x = bullet_pos.x
#         start_y = bullet_pos.y
#         for wall in walls:
#             wall_pos = wall.position
#             wall_angle = wall_pos.angle
#             if wall_angle == 0:
#                 # For horizontal walls (angle = 0)
#                 # Calculate intersection with bullet trajectory
#                 # Horizontal wall is at y = wall_pos.y
#                 if abs(math.tan(bullet_angle)) < 1e-10:  # Bullet is moving horizontally
#                     continue  # No intersection with horizontal wall
#                 # Calculate bullet trajectory parameters
#                 bullet_slope = math.tan(bullet_angle)
#                 b = start_y - bullet_slope * start_x
#                 # Calculate intersection point with horizontal wall
#                 intersection_x = (wall_pos.y - b) / bullet_slope
#                 intersection_y = wall_pos.y
#                 # Check if intersection is in front of the bullet
#                 forward_direction = (math.cos(bullet_angle) * (intersection_x - start_x) + math.sin(bullet_angle) * (intersection_y - start_y) > 0)
                    
#                 # Calculate distance to intersection
#                 distance_to_intersection = math.hypot(intersection_x - start_x, intersection_y - start_y)
                    
#                 # Check if this is the closest intersection so far
#                 if forward_direction and (not 'end_x' in locals() or distance_to_intersection < math.hypot(end_x - start_x, end_y - start_y)):
#                     end_x = intersection_x
#                     end_y = intersection_y

#         # Create a BallisticSegment object for the bullet
#         segment = BallisticSegment(bullet.shell_id, 0, bullet_pos, (end_x, end_y), bullet_angle, bullet_length, bullet_distance_to_target)
#         bullet_segments.append(segment)

#     return bullet_segments

    
async def loop(agent: Agent):
    # Your code here.
    # Here is an example of how to use the agent.
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
        bx = bullet.position.x
        by = bullet.position.y
        distance = math.hypot(px - bx, py - by)
        if distance < bullet_danger_distance:
            is_safe = False
            if math.tan(player_angle) > math.tan(bullet.position.angle):
                await agent.turn_clockwise()
            else:
                await agent.turn_counter_clockwise()
            await agent.move_forward()
            break

    if is_safe and self_info.weapon.current_bullets > 0:
        await agent.attack()

    for skill in self_info.skills:
        if skill.current_cooldown == 0:
            await agent.use_skill(skill.name)
            break
