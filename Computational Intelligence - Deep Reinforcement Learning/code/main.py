from DQN_AGENT import *
from vqallm import *


def drone_move(last_score: int, agent_p: Agent, teacher_p: VqaLlm):
    # dummy
    if performer:
        print("kounaei student")
        agent_p.move_drone()
    else:
        print("kounaei teacher")
        teacher_p.move_drone()


if __name__ == '__main__':
    # initialize agent/student
    agent = Agent(eps=1, eps_min=0.01, eps_dec=0.01, lr=0.001, input_dims=(600, 800,3), batch_size=6, n_actions=5,
                  max_mem_size=1000)
    # initialize teacher
    teacher = VqaLlm()
    # empty score eps arrays
    scores, eps_history = [0], [0.]
    # num of episodes
    episodes = 30
    print("\n")

    for i in range(episodes):
        # reset the simulation
        teacher.reset()
        score = 0
        done = False
        reward = 0
        while not done:
            # ask the teacher - get the screenshot of the position the drone was in and the action that it took
            screenshot, teacher_action = teacher.ask()

            # Convert Pillow Image to NumPy array
            screenshot = np.array(screenshot, dtype=np.int8)
            if agent.input_dims[2] == 3:
                # Remove alpha channel (if it exists)
                screenshot = screenshot[:,:,:3]
            # Assert AirSim image and expected image dimensions
            if screenshot.shape != agent.input_dims:
                error_msg = "\033[31m{}\033[0m".format(f"ERROR: AirSim returned image of shape {screenshot.shape} - DQN expected shape {agent.input_dims}")
                print(error_msg)
                raise ValueError(error_msg)

            print("Teacher action : ",teacher_action)
            # cnn takes an action based on the screenshot of the state
            cnn_action, performer = agent.choose_action(screenshot, teacher)
            print("Student action : ",cnn_action)
            # calculate the reward
            reward = teacher.calculate_reward(cnn_action, teacher_action)
            score += reward
            # either the student or the teacher moves the drone
            drone_move(score, agent, teacher, performer)
            # save the state, reward and the action that the teacher/student took
            agent.store_state(screenshot, cnn_action, reward, teacher_action)
            # agent learns
            agent.learn()
            # the drone reached the optimal state, or it has nothing else to do resets the simulation ends the episode
            if teacher_action == 4:
                done = True
                print("Score : " + str(score) + " Epsilon : " + str(agent.epsilon))
        # append score and epsilon
        scores.append(score)
        eps_history.append(agent.epsilon)
