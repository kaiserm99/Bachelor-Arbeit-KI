#!/usr/bin/env python3

import time
import sys
import numpy as np
import argparse


np.random.seed = 200

from flatland.envs.rail_env import RailEnv
from flatland.envs.rail_generators import rail_from_file
from flatland.utils.rendertools import RenderTool, AgentRenderVariant
from flatland.envs.malfunction_generators import MalfunctionParameters, ParamMalfunctionGen

import libFlatlandCBS as FlatlandCBS


"""
Debug Environments:

Head collision: Map 4, Speed 1, Agents 8
Applicable -> inital: Map 4, Speed 1, Agents 9
"""


direction_to_str = {0: "North", 1: "East", 2: "South", 3: "West"}
action_to_str = {0 : "Do nothing", 1: "Left", 2 : "Forward", 3 : "Right", 4 : "Stop moving"}


def set_speed(env, speed_ration_map):

    for a in env.agents:
        a.speed_data['speed'] = np.random.choice(list(speed_ration_map.keys()), p=list(speed_ration_map.values()))




parser = argparse.ArgumentParser()
parser.add_argument("-l", "--level", help="Insert level number", type=int)
parser.add_argument("-a", "--agents", help="Insert agent number", type=int)


args = parser.parse_args()


speed_ration_map = {1. / 1.: .0,
                    1. / 2.: .0,
                    1. / 3.: 0,
                    1. / 4.: 1}

stochastic_data = MalfunctionParameters(
                  malfunction_rate=1/40,
                  min_duration=3,
                  max_duration=4)

time_start = time.time()
env = RailEnv(
    width=0,
    height=0,
    #Bachelor-Arbeit-KI/scratch/test-envs/Test_
    rail_generator=rail_from_file("../../scratch/test-envs/Test_" + str(args.level) + "/Level_0.pkl"),
    number_of_agents=args.agents,
    random_seed=7,
    record_steps=True
)



_, info = env.reset()
# set_speed(env, speed_ration_map)



cbs = FlatlandCBS.FlatlandCBS(env)
print(f"Created an reseted the Environment in {time.time()-time_start:5f}sec\n")

time_start = time.time()
status = FlatlandCBS.mainSearch(cbs)

if status == 0:
    print(f"\nFound a solution in {time.time()-time_start:5f}sec\n")
    print(cbs.getStatistic())

else:
    print(f"\nFound no valid solution after {time.time()-time_start:5f}sec\n")
    sys.exit(1)


action_dict = FlatlandCBS.getActions(cbs)

print(action_dict)





render = False
debug = False


try:

    if render: 
        env_renderer = RenderTool(env, screen_width=2000, screen_height=2000, show_debug=True)

        env_renderer.render_env(show=True, frames=False, show_observations=False, show_predictions=False, show_rowcols=True)


    current_actions = {}
    for step in range(2000):

        if render:
            env_renderer.render_env(show=True, frames=False, show_observations=False, show_predictions=False)
            time.sleep(0.2)
            # input("Weiter?")

        for handle, agent in enumerate(env.agents):

            if debug: print(f"{handle}: ", end="")

            if agent.position is not None:
                if debug: print(f"{agent.position}, {direction_to_str[agent.direction]} -> ", end="")

            if (env.action_required(agent)):

                action = action_dict[handle].pop(0)
                
                if debug: print(action_to_str[action])
                current_actions[handle] = action

            else:
                if debug: print("+--+")

            

        obs, all_rewards, done, info = env.step(current_actions)


        if debug: print(f"[{step:3}] In goal: {[handle for handle, status in done.items() if status]}")
        

        

        if done["__all__"]:
            print(f"\nAll Agents are in their targets! After {step} iterations.")
            print(f"\nFound a solution in {time.time()-time_start:5f}sec\n")
            print(cbs.getStatistic())
            print(env.cur_episode)
            break

finally:
    if render : env_renderer.close_window()
