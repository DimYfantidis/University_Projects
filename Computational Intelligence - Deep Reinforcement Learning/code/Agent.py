import gym
import math
import random
import pandas as pd
import matplotlib.pyplot as plt
from collections import namedtuple, deque
from PIL import Image
from itertools import count
from EnvWrapper import AirSimEnv
from datetime import datetime
from typing import Literal
from colorit import *


import torch
import pickle
import torchvision
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F


# IS_PYTHON = 'inline' in matplotlib.get_backend()
# if IS_PYTHON:
#     from IPython import display
# plt.ion()

ACTION_SPACE = ["Move closer", "Move back", "Move right", "Move left", "I know enough"]

class UnrealDroneAgent:
    Transition = namedtuple('Transition', ('state', 'action', 'next_state', 'reward'))
    
    class ReplayMemory(object):
        def __init__(self, capacity: int):
            self.memory = deque([], maxlen=capacity)

        def push(self, *args):
            """Save a transition"""
            self.memory.append(UnrealDroneAgent.Transition(*args))

        def sample(self, batch_size):
            return random.sample(self.memory, batch_size)

        def __len__(self):
            return len(self.memory)


    class DQN(nn.Module):
        def __init__(self, obs_dim: torch.Size, n_actions: int):
            self.WIDTH, self.HEIGHT, self.CHANNELS = obs_dim
            # build the cnn
            super(UnrealDroneAgent.DQN, self).__init__()
            self.n_actions = n_actions
            self.conv1 = nn.Conv2d(in_channels=self.CHANNELS, out_channels=30, kernel_size=(3, 3), stride=3)
            self.maxPool1 = nn.MaxPool2d(kernel_size=(2, 2), stride=1)
            self.conv2 = nn.Conv2d(in_channels=30, out_channels=60, kernel_size=(3, 3), stride=3)
            self.maxPool2 = nn.MaxPool2d(kernel_size=(2, 2))
            self.conv3 = nn.Conv2d(in_channels=60, out_channels=90, kernel_size=(3, 3), stride=3)
            self.maxPool3 = nn.MaxPool2d(kernel_size=(2, 2))
            self.flat = nn.Flatten()

            x = torch.empty(1, self.WIDTH, self.HEIGHT, self.CHANNELS, dtype=torch.float32)
            x = x.permute(0, 3, 1, 2)
            x = self.conv1(x)
            x = self.maxPool1(x)
            x = self.conv2(x)
            x = self.maxPool2(x)
            x = self.conv3(x)
            x = self.maxPool3(x)
            x = self.flat(x)

            # Why the fuck am I supposed to keep track of the previous layer's output dimensions?
            self.fc1 = nn.Linear(x.shape[1], 128)
            self.fc2 = nn.Linear(128, 64)
            self.fc3 = nn.Linear(64, n_actions)


        def forward(self, screenshot: torch.Tensor):
            # forward
            # if not isinstance(screenshot, torch.Tensor):
            #     error_msg = "\033[31m{}\033[0m".format("WARNING: `UnrealDroneAgent.DQN.forward` must be called with `torch.Tensor` inference data")
            #     print(error_msg)
            #     raise ValueError(error_msg)
            
            screenshot = (screenshot / 255).permute(0, 3, 1, 2)
            x = F.relu(self.conv1(screenshot))
            x = self.maxPool1(x)
            x = F.relu(self.conv2(x))
            x = self.maxPool2(x)
            x = F.relu(self.conv3(x))
            x = self.maxPool3(x)
            x = self.flat(x)
            x = F.relu(self.fc1(x))
            x = F.relu(self.fc2(x))
            actions = self.fc3(x)

            return actions

    
    def __init__(
        self,
        env: gym.Env, 
        batch_size: int = 128, 
        gamma: float = 0.99,
        eps_start: float = 0.9,
        eps_decay: float = 1000.0,
        eps_end: float = 0.05,
        tau: float = 5e-3,
        learning_rate: float = 1e-4,
        replay_buffer_size: int = 10000,
        device: str = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    ): 
        # Unreal environment (hopefully) (hahahaha)
        self.env: AirSimEnv = env
        # Hyperparameters
        self.BATCH_SIZE = batch_size
        self.GAMMA = gamma
        self.EPS_START = eps_start
        self.EPS_DECAY = eps_decay
        self.EPS_END = eps_end
        self.TAU = tau
        self.LEARN_RATE = learning_rate
        print(type(self.env))
        # Loading & Execution device
        self.device = device
        # State-Action info
        self.n_actions = env.action_space.n
        self.state, self.info = env.reset()
        self.state: torch.Tensor = self.state
        # Image dimensions (+ channels)
        self.obs_dim = self.state.shape
        # Value network and Action Network
        self.policy_net = UnrealDroneAgent.DQN(self.obs_dim, self.n_actions).to(self.device)
        self.target_net = UnrealDroneAgent.DQN(self.obs_dim, self.n_actions).to(self.device)
        self.target_net.load_state_dict(self.policy_net.state_dict())
        # Value network optimizer
        self.optimizer = optim.AdamW(self.policy_net.parameters(), lr=self.LEARN_RATE, amsgrad=True)
        self.memory = UnrealDroneAgent.ReplayMemory(replay_buffer_size)
        self.steps_done = 0
        self.num_train_episodes = 0
        self.episode_durations = []

        self.reward_performance = []
        self.epsilon_values = []


    def select_action(self, state: torch.Tensor):
        # Value between [0, 1)
        sample = random.random()
        eps_threshold = self.EPS_END + (self.EPS_START - self.EPS_END) * \
            math.exp(-1. * self.steps_done / self.EPS_DECAY)
        self.epsilon_values.append((self.steps_done, eps_threshold))
        self.steps_done += 1
        if sample > eps_threshold:
            with torch.no_grad():
                # t.max(1) will return the largest column value of each row.
                # second column on max result is index of where max element was
                # found, so we pick action with the larger expected reward.
                return self.policy_net(state).max(1).indices.view(1, 1)
        else:
            return torch.tensor([[self.env.teacher.ask(False)[1]]], device=self.device, dtype=torch.long)


    def decide(self, state: torch.Tensor) -> torch.Tensor:
        with torch.no_grad():
            return self.policy_net(state).max(1).indices.view(1, 1)


    def plot_durations(self, show_result=False):
        plt.figure(1)
        durations_t = torch.tensor(self.episode_durations, dtype=torch.float)
        if show_result:
            plt.title('Result')
        else:
            plt.clf()
            plt.title('Training...')
        plt.xlabel('Episode')
        plt.ylabel('Duration')
        plt.plot(durations_t.numpy())
        # Take 100 episode averages and plot them too
        if len(durations_t) >= 100:
            means = durations_t.unfold(0, 100, 1).mean(1).view(-1)
            means = torch.cat((torch.zeros(99), means))
            plt.plot(means.numpy())

        plt.pause(0.001)  # pause a bit so that plots are updated
        # if 'inline' in matplotlib.get_backend():
        #     if not show_result:
        #         display.display(plt.gcf())
        #         display.clear_output(wait=True)
        #     else:
        #         display.display(plt.gcf())       


    def optimize_model(self):
        if len(self.memory) < self.BATCH_SIZE:
            return
        transitions = self.memory.sample(self.BATCH_SIZE)
        # Transpose the batch (see https://stackoverflow.com/a/19343/3343043 for
        # detailed explanation). This converts batch-array of Transitions
        # to Transition of batch-arrays.
        batch = UnrealDroneAgent.Transition(*zip(*transitions))

        # Compute a mask of non-final states and concatenate the batch elements
        # (a final state would've been the one after which simulation ended)
        non_final_mask = torch.tensor(tuple(map(lambda s: s is not None,
                                            batch.next_state)), device=self.device, dtype=torch.bool)
        non_final_next_states = torch.cat([s for s in batch.next_state
                                                    if s is not None])
        state_batch = torch.cat(batch.state)
        action_batch = torch.cat(batch.action)
        reward_batch = torch.cat(batch.reward)

        # Compute Q(s_t, a) - the model computes Q(s_t), then we select the
        # columns of actions taken. These are the actions which would've been taken
        # for each batch state according to policy_net
        state_action_values = self.policy_net(state_batch).gather(1, action_batch)

        # Compute V(s_{t+1}) for all next states.
        # Expected values of actions for non_final_next_states are computed based
        # on the "older" target_net; selecting their best reward with max(1).values
        # This is merged based on the mask, such that we'll have either the expected
        # state value or 0 in case the state was final.
        next_state_values = torch.zeros(self.BATCH_SIZE, device=self.device)
        with torch.no_grad():
            next_state_values[non_final_mask] = self.target_net(non_final_next_states).max(1).values
        # Compute the expected Q values
        expected_state_action_values = (next_state_values * self.GAMMA) + reward_batch

        # Compute Huber loss
        criterion = nn.SmoothL1Loss()
        loss = criterion(state_action_values, expected_state_action_values.unsqueeze(1))

        # Optimize the model
        self.optimizer.zero_grad()
        loss.backward()
        # In-place gradient clipping
        torch.nn.utils.clip_grad_value_(self.policy_net.parameters(), 100)
        self.optimizer.step()


    def train(self, num_episodes: int):
        for i_episode in range(num_episodes):
            # Initialize the environment and get its state
            print("reset train start")
            state, info = self.env.reset()
            print("reset train end")
            state = torch.tensor(state, dtype=torch.float32, device=self.device).unsqueeze(0)
            for t in count():
                action = self.select_action(state)
                print(color(f"Agent's action: {ACTION_SPACE[int(action)]}", Colors.green))
                
                screenshot, reward, terminated, truncated, _ = self.env.step(action.item())
                reward = torch.tensor([reward], device=self.device)
                done = terminated or truncated
                next_state = None if terminated else torch.tensor(screenshot, dtype=torch.float32, device=self.device).unsqueeze(0)
                # Store the transition in memory
                self.memory.push(state, action, next_state, reward)
                # Move to the next state
                state = next_state
                # Perform one step of the optimization (on the policy network)
                self.optimize_model()
                # Soft update of the target network's weights
                # θ′ ← τ * θ + (1 − τ) * θ′
                target_net_state_dict = self.target_net.state_dict()
                policy_net_state_dict = self.policy_net.state_dict()
                for key in policy_net_state_dict:
                    target_net_state_dict[key] = policy_net_state_dict[key] * self.TAU + target_net_state_dict[key] * (1-self.TAU)
                self.target_net.load_state_dict(target_net_state_dict)
                if done:
                    # self.episode_durations.append(t + 1)
                    # self.plot_durations()
                    break
            self.num_train_episodes += 1
            print(color(drone.env.teacher.llm.ask("!CHAT_RESET") + f' at the end of episode #{i_episode}', Colors.orange))


    def test(self) -> tuple[torch.Tensor, torch.Tensor]:
        # Initialize the environment and get its state
        state, info = self.env.reset()
        state = torch.tensor(state, dtype=torch.float32, device=self.device).unsqueeze(0)
        cummulative_reward = torch.tensor([0], device=self.device)
        for t in count():
            action = self.decide(state)
            screenshot, reward, terminated, truncated, _ = self.env.step(action.item())
            cummulative_reward += torch.tensor([reward], device=self.device)
            done = terminated or truncated
            next_state = None if terminated else torch.tensor(screenshot, dtype=torch.float32, device=self.device).unsqueeze(0)
            # Move to the next state
            state = next_state
            if done:
                break
        # Returns the reward sum and the final state
        self.reward_performance.append((self.num_train_episodes, int(cummulative_reward)))
        return cummulative_reward, state


    def export_results(self) -> None:
        file_path = f"./RL_results/{datetime.now().strftime("%Y-%m-%d T+%H-%M-%S")}"
        os.mkdir(file_path)
        
        open(f"{file_path}/reward_performance.pkl", "w").close()
        with open(f"{file_path}/reward_performance.pkl", "wb") as fp:
            pickle.dump(self.reward_performance, fp)
        
        open(f"{file_path}/DQN.torch", "w").close()
        torch.save(self.policy_net.state_dict(), f"{file_path}/DQN.torch")
        
        open(f"{file_path}/epsilon_values.pkl", "w").close()
        with open(f"{file_path}/epsilon_values.pkl", "wb") as fp:
            pickle.dump(self.epsilon_values, fp)


if __name__ == '__main__':
    drone = UnrealDroneAgent(
        env=AirSimEnv(),
        device=torch.device("cuda:0")
    )

    for checkpoint in range(5):
        print(color(">>>>>> Now entering TRAINING mode <<<<<<", Colors.purple))
        drone.train(num_episodes=5)
        print(color(drone.env.teacher.llm.ask("!CHAT_RESET"), Colors.orange))
        print(color(">>>>>> Now entering EVALUATION mode <<<<<<", Colors.purple))
        drone.test()
        print(color(drone.env.teacher.llm.ask("!CHAT_RESET"), Colors.orange))
        drone.export_results()
    drone.env.teacher.llm.disconnect()
