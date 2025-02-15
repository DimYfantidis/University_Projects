import os
import sys
import socket
import threading
import subprocess as sp

if not "HF_HOME" in os.environ.keys():
    os.environ["HF_HOME"] = '/media/data/cidl_llm/models_cache/'

from llm_init import llama3_init, phi3_128k_init, phi3_4k_init, llama3_quant_init
from typing import Callable
from time import time, sleep
from datetime import datetime
from platform import python_version
from torch import cuda, backends, version
from transformers import Pipeline
from colorit import *


write_mutex = threading.Lock()
def printts(*args, **kwargs):
    with write_mutex:
        print(*args, **kwargs)


class LargeLanguageModelServer:
    def __init__(
        self, 
        address: str, 
        port: int,
        access_token: str, 
        system_prompt_file: str,
        model_loader: Callable[[str, str], tuple[Pipeline, dict, list[dict[str, str]], Callable[[str], str]]],
        msg_format: str='utf-8',
        header_bufsize: int=8192,
        disconnect_message: str="!DISCONNECT",
        restart_message: str="!CHAT_RESET"
    ):
        self.is_online = True
        self.ADDRESS = address
        self.PORT = port
        self.HEADER = header_bufsize
        self.FORMAT = msg_format
        self.DISCONNECT_MESSAGE = disconnect_message
        self.RESTART_MESSAGE = restart_message
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind((self.ADDRESS, self.PORT))
        self.client_list = []
        self.system_prompt_file = system_prompt_file
        self.text_generator, \
        self.generator_call_kwargs, \
        self.message_history, \
        self.response_parser_func = model_loader(access_token, system_prompt_file)

    
    # Returns GPU's memory usage stats in MiB
    @staticmethod 
    def get_gpu_usage() -> tuple[int, int, int]:
        free = [int(x.split()[0]) for x in sp.check_output("nvidia-smi --query-gpu=memory.free --format=csv".split()).decode('ascii').split('\n')[1:-1]]
        total = [int(x.split()[0]) for x in sp.check_output("nvidia-smi --query-gpu=memory.total --format=csv".split()).decode('ascii').split('\n')[1:-1]]
        reserved = [t - f for t, f in zip(total, free)]
        return free, reserved, total
    

    def handle_client(self, conn: socket.socket, addr: tuple):
        printts(f"> Connection with {addr} established")
        while self.is_online:
            _, reserved, total = LargeLanguageModelServer.get_gpu_usage()
            print(f"> [DEBUG @ {datetime.now().strftime('%H:%M:%S')}] - GPU usage")
            for i, (r, t) in enumerate(zip(reserved, total)):
                print(f"\t> cuda:{i} usage: {r}/{t} MiB")
            cuda.empty_cache()
            try:
                # Blocks since client sends a message
                msg_length = conn.recv(self.HEADER).decode(self.FORMAT)
                has_context = conn.recv(self.HEADER).decode(self.FORMAT)
                if msg_length:
                    msg_length = int(msg_length)
                    has_context = int(has_context)
                    print(color_ansi(f"\t> Context Window: {'Saving' if has_context else 'Discarding'} interaction", ColorsFG.Yellow))
                    msg = conn.recv(msg_length).decode(self.FORMAT)
                    if msg == self.DISCONNECT_MESSAGE:
                        break
                    if msg == self.RESTART_MESSAGE:
                        self.message_history = self.message_history[:2]
                        conn.send("Chat has been reset".encode(self.FORMAT))
                        continue
                    printts(f"[{addr}] prompted with a message of {msg_length} characters")
                    # ------------------ LLaMa segment ------------------
                    self.message_history.append({"role" : "user", "content" : msg})
                    prompt = self.text_generator.tokenizer.apply_chat_template(
                        self.message_history, 
                        tokenize=False, 
                        add_generation_prompt=True
                    )
                    print(f"> [DEBUG @ {datetime.now().strftime('%H:%M:%S')}] - Prompt type: {type(prompt)}")
                    print(f"> [DEBUG @ {datetime.now().strftime('%H:%M:%S')}] - Prompt size: {sys.getsizeof(prompt)}")
                    print(f"> [DEBUG @ {datetime.now().strftime('%H:%M:%S')}] - Pipeline size: {sys.getsizeof(self.text_generator)}")
                    prompt_time = time()
                    sequences = self.text_generator(
                        prompt,
                        **self.generator_call_kwargs
                    )
                    response_time = time()
                    response: str = self.response_parser_func(sequences[0]["generated_text"])
                    if has_context:
                        self.message_history.append({"role" : "assistant", "content" : response})
                    # ------------------ LLaMa segment ------------------
                    printts(f"Prompt processed after {response_time - prompt_time:.3f} seconds)")
                    conn.send(response.encode(self.FORMAT))
            except KeyboardInterrupt:
                printts(f"Shutting Down Connection with {addr}...")
                self.is_online = False
                conn.close()
            except Exception as e:
                printts(f"Unexpected error: {e}")


    # def start(self) -> None:
    #     self.server.listen()
    #     printts(f"> Server: Now listening on {self.ADDRESS}")
    #     while True:
    #         # Blocks until a client connects
    #         conn, addr = self.server.accept()
    #         thread = threading.Thread(target=self.handle_client, args=(conn, addr))
    #         self.client_list.append({
    #             "connection" : conn, "address" : addr, "thread" : thread
    #         })
    #         thread.start()
    #         printts(f"> Active Connectons: {len(self.client_list)}")


    def start(self):
        self.server.listen()
        printts(f"> Server: Now listening on {self.ADDRESS}")
        # Blocks until the client connects
        connection, address = self.server.accept()
        self.handle_client(connection, address)


    def terminate(self):
        self.is_online = False
        self.server.shutdown(socket.SHUT_RDWR)
        self.server.close()
        printts("[SERVER] Shutting down...")


if __name__ =='__main__':
    printts(f"Python version: {python_version()}")
    printts(f"GPU support: {cuda.is_available()}")
    printts(f"Device name: {cuda.get_device_properties('cuda').name}")
    printts(f"FlashAttention available: {backends.cuda.flash_sdp_enabled()}")
    printts(f"torch version: {version.__version__}")

    llm = LargeLanguageModelServer(
        access_token="<INSERT-TOKEN-HERE-[OPTIONAL]>",
        system_prompt_file="./Large_Models_Dialogue_for_Active_Perception/llm-vqa_dialogue/system_prompts/rules_prompt_v2.txt",
        model_loader=llama3_quant_init,
        address=socket.gethostbyname(socket.gethostname()),
        port=5050
    )
    llm.start()
    llm.terminate()
