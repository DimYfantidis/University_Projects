import numpy as np 

from typing import List, Tuple
from AirSimServer import *
from gym import Env,spaces
from vqallm import VqaLlm
from PIL import Image
import torch


class AirSimEnv(Env):
    def __init__(self) -> None:
        super().__init__()
        self.action_space = spaces.Discrete(5)
        self.observation_space = spaces.Box(low = 0, high=255, shape=(3,800,600),dtype=np.uint8)
        self.teacher = VqaLlm()
        self.aw = AirsimServer()

    def step(self, action: int, has_context: bool=True) -> Tuple[spaces.Box | float | bool | dict]:
        done = False
        _, teacher_action = self.teacher.ask( has_context)
        reward = self.teacher.calculate_reward(action,teacher_action)

        if action == 4:
            done = True

        #move drone -- or here
        self.teacher.move_drone(action)
        if(self.aw.get()):
            screenshot = Image.open("state.png").convert("RGB") # state
            print("Screenshot received")
        else:
            print("No screenshot")
        #screenshot = Image.open("5.png").convert("RGB") # state

        info = {}

        return torch.from_numpy(np.array(screenshot)), reward, done, False, info

    def render(self):
        pass

    def reset(self) -> Tuple[spaces.Box | dict]:
        screenshot = self.teacher.reset()
        print("ekana reset")
        return torch.from_numpy(np.array(screenshot)),{}

    def close(self):
        return super().close()