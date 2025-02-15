from VQAWrapper import VqaWrapper
from AirSimServer import *
from colorit import *
from PIL import Image
from llm_client import LargeLanguageModelClient
import socket
import random
import re


# def actr() -> str:
#     r = random.randint(0,3)
#     if  r == 0 :
#         return "Move closer"
#     elif r == 1 :
#         return "Move back"
#     elif r == 2 :
#         return "Move right"
#     elif r == 3 :
#         return "Move left"
#     elif r == 4 :
#         return "I know enough"


class VqaLlm:
    def __init__(self):
        # initialize llm
        #print("connecting...")
        self.llm = LargeLanguageModelClient(
            remote_address=socket.gethostbyname(socket.gethostname()), 
            remote_port=5050
        ).connect()
        # initialize vqa
        self.vqa = VqaWrapper()
        # initialize AirSim wrapper
        self.aw = AirsimServer()
        # last action taken
        self.action = 0
        # last question asked to vqa
        self.question = "what do you see?"

    def ask(self):
        # take screenshot using the AirSim client
        # if(self.aw.get()):
        #     screenshot = Image.open("state.png").convert("RGB") # state
        #     print("Screenshot received")
        # else:
        #     print("No screenshot")

        screenshot = Image.open("5.png").convert("RGB") # state
        # ask vqa based on the screenshot and the question
        answer, caption, conf, _ = self.vqa.ask(screenshot, self.question)
        # format the answer
        vqa_answer = f"Answer: {answer}\n Caption: [{caption}]"
        vqa_answer_formated = f"Answer: {answer} Caption: [{caption}]"
        print("vqa said : " + vqa_answer_formated)
        # ask llm to give the next action/question for the vqa based on its answer
        #x = "i see "+re.findall("\'.*\'",caption)[0][1:-1]
        #print("llm hears : " + x)
        conv = self.llm.ask(vqa_answer)
        # conv = actr() + " " +conv
        print("llm answer : ", color_ansi(conv, ColorsFG.BrightBlue))
        # format the llm answer
        try:
            start_question = conv.find('"')
            end_question = conv.find('"', start_question + 1)
            if start_question != -1 and end_question != -1:
                self.question = conv[start_question + 1:end_question]
        except (IndexError, ValueError):
            self.question = conv
            print(self.question)

        # According to llm answer save the action that it says
        if "Move closer" in conv:
            #print("mpika sto 0")
            self.action = 0
        elif "Move back" in conv:
            #print("mpika sto 1")
            self.action = 1
        elif "Move right" in conv:
            #print("mpika sto 2")
            self.action = 2
        elif "Move left" in conv:
            #print("mpika sto 3")
            self.action = 3
        elif "I know enough" in conv or "Danger detected" in conv:
            #print("mpika sto 4")
            self.action = 4
        else:
            print("ERR : no action")

        # return the screenshot and the action taken
        #print("girnaei action o teacher")
        return screenshot, self.action

    def move_drone(self):
        # move the drone according the action that the teacher said
        print("teacher send action : " + str(self.action) + "\n")
        self.aw.send(self.action)

    def calculate_reward(self, cnn_action: int, teacher_action: int) -> int:
        # calculate reward
        # work in progress
        if cnn_action == teacher_action:
            return 0
        else:
            return -1

    def reset(self):
        # reset the simulation state to the default first question
        self.action = 9
        self.aw.send(9)
        self.question = "what do you see?"
