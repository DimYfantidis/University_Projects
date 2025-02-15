import os
import time


class AirsimServer:
    def __init__(self):
        self.counter = 0 
        pass

    def get(self):
        counter = 0
        #if os.path.isfile('/media/data/cidl_llm/state.png'):
        #    os.remove("state.png")
        control = True
        while control:
            if os.path.isfile('/media/data/cidl_llm/screenshot.png'):
                os.rename('screenshot.png', 'state.png')
                os.system(f"cp state.png DRONE_CAMERA/observation_{self.counter}.png")
                self.counter += 1
                time.sleep(2)
                return True
            elif os.path.isfile('/media/data/cidl_llm/state.png'):
                time.sleep(2)
                return True
            else:
                print("No new state, server sleeping...")
                time.sleep(5)
                counter += 1
                if counter > 20:
                    print("varethika")
                    exit()

    def send(self, action : int):
        f = open("action.txt", "w")
        f.write(str(action))
        f.close()
