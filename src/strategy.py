# Strategy classes for AI behavior
import math
import heapq
from agent.agent import Agent
from logic import judge_bullet, find_rival, target_rival

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
