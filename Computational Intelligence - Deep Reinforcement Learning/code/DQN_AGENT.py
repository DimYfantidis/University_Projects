import torch as T
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as opt
import numpy as np

from vqallm import VqaLlm
from PIL import Image
from torch.nn import Conv2d
from torch.nn import Linear
from torch.nn import MaxPool2d
from AirSimServer import *
from colorit import *
from typing import Callable


class DQN(nn.Module):
    def __init__(self, lr: float, n_actions: int, input_dims: tuple[int, int, int], device: str):
        self.WIDTH, self.HEIGHT, self.CHANNELS = input_dims
        self.device = device
        # build the cnn
        super(DQN, self).__init__()
        self.n_actions = n_actions
        self.conv1 = Conv2d(in_channels=self.CHANNELS, out_channels=30, kernel_size=(3, 3), stride=3, device=self.device)
        self.maxPool1 = MaxPool2d(kernel_size=(2, 2), stride=1)
        self.conv2 = Conv2d(in_channels=30, out_channels=60, kernel_size=(3, 3), stride=3, device=self.device)
        self.maxPool2 = MaxPool2d(kernel_size=(2, 2))
        self.conv3 = Conv2d(in_channels=60, out_channels=90, kernel_size=(3, 3), stride=3, device=self.device)
        self.maxPool3 = MaxPool2d(kernel_size=(2, 2))
        self.flat = nn.Flatten()

        x = T.empty(1, self.WIDTH, self.HEIGHT, self.CHANNELS, dtype=T.float32).to(self.device)
        x = x.permute(0, 3, 1, 2)
        #print(x.shape)
        x = self.conv1(x)
        #print(x.shape)
        x = self.maxPool1(x)
        #print(x.shape)
        x = self.conv2(x)
        #print(x.shape)
        x = self.maxPool2(x)
        #print(x.shape)
        x = self.conv3(x)
        #print(x.shape)
        x = self.maxPool3(x)
        #print(x.shape)
        x = self.flat(x)
        #print(x.shape)
        
        # self.fc1 = Linear(60 * 3 * 3, 128)
        self.fc1 = Linear(x.shape[1], 128, device=self.device)
        self.fc2 = Linear(128, 64, device=self.device)
        self.fc3 = Linear(64, n_actions, device=self.device)
        # optimizer : Adam //// Loss : cross entropy
        self.optimizer = opt.Adam(self.parameters(), lr=lr)
        self.loss = nn.CrossEntropyLoss()


    def forward(self, screenshot: np.ndarray[np.int8] | Image.Image | T.Tensor):
        # forward
        if isinstance(screenshot,  Image.Image):
            print("\033[31m{}\033[0m".format("WARNING: DQN forward() called with `PIL.Image.Image` data - Consider inferring input as `torch.Tensor`"))
            screenshot = np.array(screenshot, dtype=np.float32)
        if isinstance(screenshot, np.ndarray):
            print("\033[31m{}\033[0m".format("WARNING: DQN forward() called with `numpy.ndarray` data - Consider inferring input as `torch.Tensor`"))
            screenshot = T.from_numpy([screenshot])#.to(device=self.device)

        screenshot = (screenshot / 255).permute(0, 3, 1, 2)
        x = F.relu(self.conv1(screenshot))
        x = self.maxPool1(x)
        x = F.relu(self.conv2(x))
        x = self.maxPool2(x)
        x = F.relu(self.conv3(x))
        x = self.maxPool3(x)
        #x = T.flatten(x, 1)
        x = self.flat(x)
        x = F.relu(self.fc1(x))
        x = F.relu(self.fc2(x))
        actions = self.fc3(x)

        return actions


class Agent():
    def __init__(self, eps: float, eps_min: float, eps_dec: float, lr, input_dims: tuple[int, int, int], batch_size: int, n_actions: int, max_mem_size: int):
        # initialize AirSim wrapper
        self.aw = AirsimServer()
        self.device = T.device('cuda:1' if T.cuda.is_available() else 'cpu')
        # initialize agent
        # screenshot dims
        self.input_dims = input_dims
        # initial - minimum - decrease rate of epsilon
        self.epsilon = eps
        self.epsilon_min = eps_min
        self.epsilon_decrease = eps_dec
        # learning rate
        self.lr = lr
        # num of output actions
        self.action_space = n_actions
        # maximum stored states
        self.mem_size = max_mem_size
        # batch size
        self.batch_size = batch_size
        # memory pointer
        self.mem_counter = 0
        # initialize a DQN for the agent to use
        self.dqn = DQN(self.lr, self.action_space, self.input_dims, self.device).to(self.device)
        # arrays to store state - action and related info
        self.state_memory = np.zeros((self.mem_size, *self.input_dims), dtype=np.int8)
        self.action_memory = np.zeros((self.mem_size,), dtype=np.int8)
        self.target_memory = np.zeros((self.mem_size,), dtype=np.int8)
        self.reward_memory = np.zeros((self.mem_size,), dtype=np.int8) # DTYPE may change
        #stores last action
        self.last_action = 0


    def move_drone(self) -> None:
        # move the drone according the action that the student said
        print("teacher send action : " + str(self.last_action) + "\n")
        self.aw.send(self.last_action)


    def store_state(self, state: np.ndarray, action: int, reward: int, target: int) -> None:
        # stores info for state, actions taken and reward
        index = self.mem_counter % self.mem_size
        self.state_memory[index] = state
        self.action_memory[index] = action
        self.reward_memory[index] = reward
        self.target_memory[index] = target
        self.mem_counter += 1


    def choose_action(self, screenshot: np.ndarray, teacher: VqaLlm) -> tuple[int, bool]:
        # chooses the next action
        # random or based on the output of the cnn
        if np.random.random() > self.epsilon:
            state = T.tensor([screenshot]).to(self.device)
            actions = self.dqn.forward(state)
            action = int(T.argmax(actions).item())
            performer = True
        else:
            performer = False
            _, action = teacher.ask()
            # action = np.random.choice(self.action_space)

        self.last_action = action
        return action, performer


    def learn(self) -> None:
        # if the agent has enough data
        if self.mem_counter < self.batch_size:
            return
        
        print("Start training..")
        self.dqn.optimizer.zero_grad()
        max_mem = min(self.mem_size, self.mem_counter)
        batch = np.random.choice(max_mem, self.batch_size, replace=False)
        #batch_index = np.arange(self.batch_size, dtype=np.int32)
        #print(self.state_memory.shape)
        state_batch = T.tensor(self.state_memory[batch], dtype=T.int8).to(self.device)
        #reward_batch = T.tensor(self.reward_memory[batch]).to(self.device)
        #action_batch = self.action_memory[batch]

        q_eval = self.dqn.forward(screenshot=state_batch)
        q_target = F.one_hot(T.tensor(self.target_memory[batch], dtype=T.int64), num_classes=self.dqn.n_actions).to(T.float32).to(self.device)

        #print(q_eval)
        #print(q_target)
        loss = self.dqn.loss(q_eval, q_target).to(self.device)  # How ???
        loss.backward()
        self.dqn.optimizer.step()
        print(color(loss, Colors.green))

        self.epsilon = self.epsilon - self.epsilon_decrease if self.epsilon > self.epsilon_min else self.epsilon_min
        print("Training finished.")
