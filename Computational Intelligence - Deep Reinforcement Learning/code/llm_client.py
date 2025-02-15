import socket


class LargeLanguageModelClient:
    def __init__(
        self, 
        remote_address: str, 
        remote_port: int, 
        msg_format: str='utf-8',
        server_bufsize: int=8192, 
        local_bufsize: int=8192,
        disconnect_message: str="!DISCONNECT",
        enable_chat_history: bool=False
    ):
        # Client's socket
        self._client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # IPv4 address (probably supports IPv6 addresses too) & connection port
        self._REMOTE_ADDRESS = remote_address
        self._REMOTE_PORT = remote_port
        # Network's format for sharing data, defaults to UTF-8 encoding
        self._MESSAGE_FORMAT = msg_format
        # Remote computer's header size
        self._SERVER_BUFSIZE = server_bufsize
        # Local computer's header size
        self._LOCAL_BUFSIZE = local_bufsize
        # Special token allocated for disconnecting from the server
        self._DISCONNECT_MESSAGE = disconnect_message
        # The conversation's history
        self.chat_history = [] if enable_chat_history else None


    # Establishes remote connection with the LLM's server.
    def connect(self) -> 'LargeLanguageModelClient':
        self._client.setblocking(True)
        self._client.connect((self._REMOTE_ADDRESS, self._REMOTE_PORT))
        return self


    # Prompts the LLM with the provided argument.
    def ask(self, prompt: str, has_context: bool = True) -> str:
        try:
            # Update conversation history
            if self.chat_history is not None and has_context:
                self.chat_history.append({
                    "role" : "user",
                    "content" : prompt
                })
            # Encode the prompt message
            message = prompt.encode(self._MESSAGE_FORMAT)
            msg_length = len(message)
            # Prepare the message length with padding
            send_length = str(msg_length).encode(self._MESSAGE_FORMAT)
            send_length += b' ' * (self._SERVER_BUFSIZE - len(send_length))
            # Send the length of the message followed by the message itself
            #print("Sending message length...")
            self._client.sendall(send_length)

            send_context_status = str(1 if has_context else 0).encode(self._MESSAGE_FORMAT)
            send_context_status += b' ' * (self._SERVER_BUFSIZE - len(send_context_status))
            self._client.sendall(send_context_status)
            #print("Sending message...")
            self._client.sendall(message)
            # Receive the response from the server
            #print("Waiting for response...")
            response = self._client.recv(self._LOCAL_BUFSIZE).decode(self._MESSAGE_FORMAT)
            #print("Response received")
            # Update conversation history
            if self.chat_history is not None and has_context:
                self.chat_history.append({
                    "role" : "assistant",
                    "content" : response
                })
            return response
        except socket.error as e:
            print(f"Socket error: {e}")
            return ""
        except Exception as e:
            print(f"Unexpected error: {e}")
            return ""


    # Adds the option to prompt the LLM with the contents of a text file.
    def file_prompt(self, filename: str) -> str:
        with open(filename, "r") as fp:
            text_prompt = fp.read(-1)
        return self.ask(text_prompt)     


    # Shuts down connection to the server
    def disconnect(self) -> 'LargeLanguageModelClient':
        self.ask(self._DISCONNECT_MESSAGE)
        self._client.shutdown(socket.SHUT_RDWR)
        self._client.close()
        return self



if __name__ == '__main__':
    # Initialize the remote connection for LLaMa
    llm = LargeLanguageModelClient(
        remote_address=socket.gethostbyname(socket.gethostname()),
        #remote_address=None, # My IP goes here...
        remote_port=5050
    ).connect()

    log_stream = open("./logfiles/conversation.log", "w")
    with open("./Large_Models_Dialogue_for_Active_Perception/llm-vqa_dialogue/prompts/Drone_Photography_Prompts.txt", "r") as fp:
        sample_prompts = fp.readlines()

    # Conversation loop
    try:
        # while True:
        for prompt in sample_prompts:
            #i see a fire fighter is flying over the field
            # question = input(">>>")
            question = prompt.removeprefix("Drone photo: ")[:-1]
            print(f"PROMPT: {question}")
            log_stream.write(f"PROMPT: {question}\n")
            response = llm.ask(question)
            print(f"RESPONSE: {response}", end='\n\n')
            log_stream.write(f"RESPONSE: {response}\n\n")
    except KeyboardInterrupt:
        print("LLM disconnecting...")
    finally:
        llm.disconnect()
        log_stream.close()
