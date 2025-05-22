import argparse
import asyncio
import logging
import os
from agent.stage import Stage
from agent.agent import Agent
import math
import heapq
import pygame
import threading
import time
from os import system


async def selectBuff(agent: Agent):
    # Your code here.
    # Here is an example of how to select a buff.
    # Always select the first buff in the available buff list.
    available_buffs = agent.availiable_buffs
    if available_buffs is None:
        logging.warning("No available buffs.")
        return
    await agent.select_buff(available_buffs.buffs[0].name)
class line:
    def __init__(self, start, end):
        self.start = start
        self.end = end
        self.angle = math.atan2(end[1] - start[1], end[0] - start[0])
        self.length = math.hypot(end[0] - start[0], end[1] - start[1])
    def __init__(self, start, angle, length):
        self.start = start
        self.angle = angle
        self.length = length
        self.end = (start[0] + length * math.cos(angle), start[1] + length * math.sin(angle))
    def find_intersection(line1, line2):
        # Check if two lines intersect
        x1, y1 = line1.start
        x2, y2 = line1.end
        x3, y3 = line2.start
        x4, y4 = line2.end
        denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
        if denom == 0:
            return None
        ua = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denom
        ub = ((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / denom
        if 0 <= ua <= 1 and 0 <= ub <= 1:
            intersection_x = x1 + ua * (x2 - x1)
            intersection_y = y1 + ua * (y2 - y1)
            return (intersection_x, intersection_y)
        return None
    def distance(point):
        # Check if a point is within a certain distance from the line
        x1, y1 = self.start
        x2, y2 = self.end
        px, py = point
        line_length = math.hypot(x2 - x1, y2 - y1)
        if line_length == 0:
            return math.hypot(px - x1, py - y1)
        # Calculate the projection of the point onto the line
        t = ((px - x1) * (x2 - x1) + (py - y1) * (y2 - y1)) / (line_length ** 2)
        t = max(0, min(1, t))
        closest_x = x1 + t * (x2 - x1)
        closest_y = y1 + t * (y2 - y1)
        return math.hypot(px - closest_x, py - closest_y)
class path:
    def __init__(self, points=None, lines=None):
        self.lines = []
        if points is not None and len(points) >= 2:
            for i in range(len(points) - 1):
                # Create lines from consecutive points
                start = points[i]
                end = points[i+1]
                angle = math.atan2(end[1] - start[1], end[0] - start[0])
                length = math.hypot(end[0] - start[0], end[1] - start[1])
                self.lines.append(line(start, angle, length))
        elif lines is not None:
            self.lines = lines
    
    def add_point(self, point):
        if not self.lines:
            # Store first point to use with next point
            self.start_point = point
        else:
            # Add line from last point to new point
            last_point = self.lines[-1].end
            angle = math.atan2(point[1] - last_point[1], point[0] - last_point[0])
            length = math.hypot(point[0] - last_point[0], point[1] - last_point[1])
            self.lines.append(line(last_point, angle, length))
    
    def add_line(self, line_to_add):
        self.lines.append(line_to_add)
    
    def length(self):
        return sum(l.length for l in self.lines)
    
    def intersects_with_line(self, other_line):
        for l in self.lines:
            intersection = line.find_intersection(l, other_line)
            if intersection is not None:
                return True, intersection
        return False, None
    
    def intersects_with_path(self, other_path):
        for l1 in self.lines:
            for l2 in other_path.lines:
                intersection = line.find_intersection(l1, l2)
                if intersection is not None:
                    return True, intersection
        return False, None
    def distance(self, point):
        # Check if a point is within a certain distance from the path
        dist = []
        for l in self.lines:
            dist.append((l.distance(point),l.angle))
        return min(dist,key=lambda x: x[0]) if dist else float('inf')

def judge_bullet(bullet_pos, bullet_speed, player, walls):
    bullet_angle = bullet_pos.angle
    player_pos = player.position
    is_safe = True
    dist = []
    bullet_path = path([(bullet_pos.x, bullet_pos.y)])
    for i in range(1, 10):
        bullet_line=line((bullet_pos.x, bullet_pos.y), bullet_angle, float('inf'))
        for wall in walls:
            wall_y = wall.position.y*10
            wall_x = wall.position.x*10
            wall_length = 10  # Assuming default wall length
            wall_line = line((wall_x, wall_y), wall.position.angle, wall_length)
            intersection = bullet_line.find_intersection(wall_line)
            if intersection is not None:
                dist.append((intersection,math.hypot(bullet_pos.x - intersection[0], bullet_pos.y - intersection[1]),wall_line.angle))
        if not dist:
            distance = float('inf')
        else:
            intersection,_,angle=min(dist, key=lambda x: x[1])
        bullet_path.add_point(intersection)
        distance,angle=bullet_path.distance((bullet_pos.x, bullet_pos.y))
        if bullet_path.distance((player_pos.x, player_pos.y)) < 1:
            is_safe = False
            return is_safe, angle
        bullet_angle=(2*angle-bullet_angle)%360
        bullet_pos.x= intersection[0]
        bullet_pos.y= intersection[1]
    return is_safe, None
        # Calculate the new angle after reflection
# def find_rival(walls,self_info, opponent_info):
#     """Find the shortest path to the rival player using A* algorithm."""
#     start = (int(self_info.position.x), int(self_info.position.y))
#     goal = (int(opponent_info.position.x), int(opponent_info.position.y))
#     # A* algorithm
#     open_set = {}
#     came_from = {}
#     g_score = {start: 0}
#     f_score = {start: math.sqrt((start[0] - goal[0])**2 + (start[1] - goal[1])**2)}
    
#     open_heap = [(f_score[start], start)]
#     heapq.heapify(open_heap)
    
#     directions = [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (-1, 1), (1, -1), (-1, -1)]
    
#     while open_heap:
#         _, current = heapq.heappop(open_heap)
        
#         if current == goal:
#             # Reconstruct path
#             path = [current]
#             while current in came_from:
#                 current = came_from[current]
#                 path.append(current)
#             path.reverse()
#             return path
        
#         for dx, dy in directions:
#             next_x, next_y = current[0] + dx, current[1] + dy
#             neighbor = (next_x, next_y)
            
#             # Check if move crosses any wall
#             wall_crossed = False
#             for wall in walls:
#                 wx, wy = wall.position.x*10, wall.position.y*10
#                 if wall.position.angle == 0:  # Horizontal wall
#                     # Wall from (wx-10, wy) to (wx, wy)
#                     if min(current[0], next_x) <= wx and max(current[0], next_x) >= wx-10 and (
#                         (current[1] <= wy and next_y >= wy) or (current[1] >= wy and next_y <= wy)):
#                         wall_crossed = True
#                         break
#                 elif wall.position.angle == 90:  # Vertical wall
#                     # Wall from (wx, wy-10) to (wx, wy)
#                     if min(current[1], next_y) <= wy and max(current[1], next_y) >= wy-10 and (
#                         (current[0] <= wx and next_x >= wx) or (current[0] >= wx and next_x <= wx)):
#                         wall_crossed = True
#                         break
            
#             if wall_crossed:
#                 continue
            
#             tentative_g_score = g_score[current] + math.sqrt(dx*dx + dy*dy)
            
#             if tentative_g_score < g_score.get(neighbor, float('inf')):
#                 came_from[neighbor] = current
#                 g_score[neighbor] = tentative_g_score
#                 f_score[neighbor] = tentative_g_score + math.sqrt((neighbor[0] - goal[0])**2 + (neighbor[1] - goal[1])**2)
                
#                 if neighbor not in open_set:
#                     open_set[neighbor] = True
#                     heapq.heappush(open_heap, (f_score[neighbor], neighbor))
#     return None  # No path found
def target_rival(walls,self_info, opponent_info):
    for i in range(0, 359):
        bullet_pos = self_info.position
        bullet_pos.x += 0.8 * math.cos(i/180*math.pi)
        bullet_pos.y += 0.8 * math.sin(i/180*math.pi)
        bullet_pos.angle = i
        is_safe, direction= judge_bullet(bullet_pos, bulletspeed, opponent_info, walls)
        if not is_safe:
            return i
    return 360
# Initialize Pygame visualization
pygame.init()
screen_width, screen_height = 1100,1100
screen = pygame.display.set_mode((screen_width, screen_height))
pygame.display.set_caption("THUAI8 Debugger")
clock = pygame.time.Clock()

# Colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
YELLOW = (255, 255, 0)
GRAY = (150, 150, 150)

# Scale factor for visualization (pixels per game unit)
SCALE = 10
ORIGIN_X, ORIGIN_Y =50,screen_height-50  # Center of the screen

# Game state for visualization
game_state = {
    "self_info": None,
    "opponent_info": None,
    "walls": [],
    "bullets": [],
    "path": []
}

def update_game_state(self_info, opponent_info, walls, bullets, path=None):
    game_state["self_info"] = self_info
    game_state["opponent_info"] = opponent_info
    game_state["walls"] = walls
    game_state["bullets"] = bullets
    if path:
        game_state["path"] = path

def draw_game():
    screen.fill(BLACK)
    
    # Draw walls
    if game_state["walls"]:
        for wall in game_state["walls"]:
            wx, wy = wall.position.x, wall.position.y
            wall_angle = wall.position.angle
            wx_screen = ORIGIN_X + wx * 100
            wy_screen = ORIGIN_Y - wy * 100  # Y is inverted in pygame
            
            if wall_angle == 0:  # Horizontal wall
                pygame.draw.line(screen, WHITE, 
                                (wx_screen - 10 * SCALE, wy_screen), 
                                (wx_screen, wy_screen), 3)
            elif wall_angle == 90:  # Vertical wall
                pygame.draw.line(screen, WHITE, 
                                (wx_screen, wy_screen), 
                                (wx_screen, wy_screen + 10 * SCALE), 3)
    
    # Draw path
    if game_state["path"]:
        for i in range(len(game_state["path"]) - 1):
            x1, y1 = game_state["path"][i]
            x2, y2 = game_state["path"][i + 1]
            x1_screen = ORIGIN_X + x1 * SCALE
            y1_screen = ORIGIN_Y - y1 * SCALE
            x2_screen = ORIGIN_X + x2 * SCALE
            y2_screen = ORIGIN_Y - y2 * SCALE
            pygame.draw.line(screen, YELLOW, (x1_screen, y1_screen), (x2_screen, y2_screen), 2)
    
    # Draw bullets
    if game_state["bullets"]:
        for bullet in game_state["bullets"]:
            bx, by = bullet.position.x, bullet.position.y
            bx_screen = ORIGIN_X + bx * SCALE
            by_screen = ORIGIN_Y - by * SCALE
            pygame.draw.circle(screen, RED, (int(bx_screen), int(by_screen)), 3)
    
    # Draw players
    if game_state["self_info"]:
        px, py = game_state["self_info"].position.x, game_state["self_info"].position.y
        angle = game_state["self_info"].position.angle
        px_screen = ORIGIN_X + px * SCALE
        py_screen = ORIGIN_Y - py * SCALE
        
        # Player circle
        pygame.draw.circle(screen, GREEN, (int(px_screen), int(py_screen)), 10)
        
        # Direction indicator
        dx = 20 * math.cos(angle)
        dy = 20 * math.sin(angle)
        pygame.draw.line(screen, GREEN, 
                        (int(px_screen), int(py_screen)), 
                        (int(px_screen + dx), int(py_screen - dy)), 2)
    
    if game_state["opponent_info"]:
        ox, oy = game_state["opponent_info"].position.x, game_state["opponent_info"].position.y
        ox_screen = ORIGIN_X + ox * SCALE
        oy_screen = ORIGIN_Y - oy * SCALE
        pygame.draw.circle(screen, BLUE, (int(ox_screen), int(oy_screen)), 10)
        # Direction indicator
        angle = game_state["opponent_info"].position.angle
        dx = 20 * math.cos(angle)
        dy = 20 * math.sin(angle)
        pygame.draw.line(screen, BLUE, 
                        (int(ox_screen), int(oy_screen)), 
                        (int(ox_screen + dx), int(oy_screen - dy)), 2)
    
    # Display debug info
    if game_state["self_info"]:
        global state
        debug_info = [
            f"Position: ({game_state['self_info'].position.x:.1f}, {game_state['self_info'].position.y:.1f})",
            f"Angle: {game_state['self_info'].position.angle:.1f}",
            f"Bullets: {game_state['self_info'].weapon.current_bullets}",
            f"Distance to opponent: {math.hypot(game_state['opponent_info'].position.x - game_state['self_info'].position.x, game_state['opponent_info'].position.y - game_state['self_info'].position.y):.1f}",
            f"State: {state}",
        ]
        
        for i, text in enumerate(debug_info):
            font = pygame.font.SysFont("Arial", 16)
            text_surface = font.render(text, True, WHITE)
            screen.blit(text_surface, (10, 10 + i * 20))
    
    pygame.display.flip()

def visualization_thread():
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
        
        draw_game()
        clock.tick(30)
    
    pygame.quit()
# Start visualization in a separate thread
vis_thread = threading.Thread(target=visualization_thread, daemon=True)
vis_thread.start()
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
    update_game_state(self_info, opponent_info, walls, bullets)
    is_safe = True
    # anti-rotation
    await agent.turn_clockwise(0)
    bullet_danger_distance = 5.0
    for bullet in bullets:
        state="avoiding"
        is_safe, direction = judge_bullet(bullet.position,bullet.speed, self_info, walls)
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
                diff = ( - diff) % 360
                if diff > 45:
                    await agent.turn_clockwise(45)
                else:
                    await agent.turn_clockwise(diff)
            await agent.turn_clockwise(0)
            await agent.move_forward()
            # break

    wall_safe_distance = 1.0
    if is_safe and self_info.weapon.current_bullets > 0:
        distance = math.hypot(opponent_info.position.x - px, opponent_info.position.y - py)
        state="moving"
        await agent.move_forward()
        my_line=line((px, py),player_angle, float('inf'))
        dist = []
        for wall in walls:
            wall_y = wall.position.y*10
            wall_x = wall.position.x*10
            wall_length = 10  # Assuming default wall length
            wall_line = line((wall_x, wall_y), wall.position.angle, wall_length)
            intersection = my_line.find_intersection(wall_line)
            if intersection is not None:
                dist.append((intersection,math.hypot(bullet_pos.x - intersection[0], bullet_pos.y - intersection[1]),wall_line.angle))
        if not dist:
            distance = float('inf')
        else:
            intersection,distance,angle=min(dist, key=lambda x: x[1])
        if distance < wall_safe_distance:
            await agent.turn_clockwise()
        else:
            await agent.turn_clockwise(0)
        target = target_rival(walls, self_info, opponent_info)
        if target != 360: 
            await agent.move_forward(0)
            diff = (target - int(player_angle*180/math.pi)) % 360
            while abs(diff) > 5:
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

class Options:
    def __init__(self, logging_level: int, server: str, token: str):
        self.logging_level = logging_level
        self.server = server
        self.token = token


DEFAULT_LOGGING_LEVEL = logging.INFO
DEFAULT_SERVER_ADDRESS = "ws://localhost:14514"
DEFAULT_TOKEN = "1919810"
DEFAULT_LOOP_INTERVAL = 0.2  # In seconds.
LOGGING_FORMAT = "[%(asctime)s] [%(levelname)s] %(message)s"


async def main():
    options = parse_options()

    logging.basicConfig(level=options.logging_level, format=LOGGING_FORMAT)

    agent = Agent(options.token, DEFAULT_LOOP_INTERVAL)
    logging.info(f"{agent} is starting with server {options.server}")

    await agent.connect(options.server)

    is_previous_connected = False
    is_previous_game_ready = False
    is_buff_selected = False

    while True:
        await asyncio.sleep(DEFAULT_LOOP_INTERVAL)

        if not agent.is_connected():
            if is_previous_connected:
                logging.error(f"{agent} is disconnected")
                is_previous_connected = False

            logging.debug(f"{agent} is waiting for the connection")
            continue

        if not is_previous_connected:
            logging.info(f"{agent} is connected")
            is_previous_connected = True

        if not agent.is_game_ready():
            if is_previous_game_ready:
                logging.error(f"{agent} is no longer in a ready game")
                is_previous_game_ready = False

            logging.debug(f"{agent} is waiting for the game to be ready")
            continue

        if not is_previous_game_ready:
            logging.info(f"{agent} is in a ready game")
            is_previous_game_ready = True

        if (
            agent.game_statistics is not None
            and agent.game_statistics.current_stage is Stage.REST
        ):
            if not is_buff_selected:
                await selectBuff(agent)
            logging.debug(f"{agent} is waiting for next battle")
            is_buff_selected = True
            continue

        if is_buff_selected:
            logging.info(f"{agent} is in a new battle")
            is_buff_selected = False

        await loop(agent)


def parse_options() -> Options:
    server_env = os.getenv("GAME_HOST", default=DEFAULT_SERVER_ADDRESS)
    token_env = os.getenv("TOKEN", default=DEFAULT_TOKEN)

    parser = argparse.ArgumentParser("agent")
    parser.add_argument(
        "--logging-level",
        type=int,
        help="Logging level",
        default=DEFAULT_LOGGING_LEVEL,
        choices=[
            logging.CRITICAL,
            logging.ERROR,
            logging.WARNING,
            logging.INFO,
            logging.DEBUG,
        ],
    )
    parser.add_argument("--server", type=str, help="Server address", default=server_env)
    parser.add_argument("--token", type=str, help="Agent token", default=token_env)
    args = parser.parse_args()
    return Options(
        logging_level=args.logging_level, server=args.server, token=args.token
    )


if __name__ == "__main__":
    
    asyncio.run(main())
    
