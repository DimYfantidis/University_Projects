import subprocess as sp
import os

command = "nvidia-smi --query-gpu=memory.free --format=csv"
print(int(sp.check_output(command.split()).decode('ascii').split('\n')[1].split()[0]))