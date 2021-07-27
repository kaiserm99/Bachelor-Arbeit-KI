from flatland.evaluators.client import FlatlandRemoteClient
from flatland.core.env_observation_builder import DummyObservationBuilder
from my_observation_builder import CustomObservationBuilder
from flatland.envs.observations import GlobalObsForRailEnv

from flatland.envs.agent_utils import RailAgentStatus
from flatland.envs.rail_env_shortest_paths import get_shortest_paths
from flatland.envs.malfunction_generators import malfunction_from_params,MalfunctionParameters
from flatland.envs.observations import TreeObsForRailEnv, GlobalObsForRailEnv
from flatland.envs.predictions import ShortestPathPredictorForRailEnv
from flatland.envs.rail_env import RailEnv
from flatland.envs.rail_generators import sparse_rail_generator
from flatland.envs.schedule_generators import sparse_schedule_generator
from flatland.utils.rendertools import RenderTool, AgentRenderVariant
from flatland.envs.rail_generators import rail_from_file

from libPythonCBS import PythonCBS

from typing import List, Tuple
from logging import warning

import argparse

import os
import subprocess
import numpy as np
import time
import copy

parser = argparse.ArgumentParser()
parser.add_argument("-l", "--level", help="Insert level number", type=int)
parser.add_argument("-a", "--agents", help="Insert agent number", type=int)


args = parser.parse_args()


#####################################################################
# parameter trigger local testing and remote testing
#####################################################################

remote_test = False
save_txt_file = False
debug_print = True
env_renderer_enable = False
input_pause_renderer = False

agent_priority_strategy = 3  #  the strategy for sorting agents, choosing a number between 0 and 5
neighbor_generation_strategy = 3    # 0: random walk; 1: start; 2: intersection; 3: adaptive; 4: iterative
debug = False
framework = "LNS"  # "LNS" for large neighborhood search or "Parallel-LNS" for parallel LNS.
time_limit = 200  #Time limit for computing initial solution.
default_group_size = 5  # max number of agents in a group for LNS
stop_threshold = 30
max_iteration = 1000 # when set to 0, the algorithm only run prioritized planning for initial solution.
agent_percentage = 1.1 # >1 to plan all agents. Otherwise plan only certain percentage of agents.
replan = True # turn on/off partial replanning.
replan_timelimit = 3.0 # Time limit for replanning.



#####################################################################
# Instantiate a Remote Client
#####################################################################


#####################################################################
# Main evaluation loop
#
# This iterates over an arbitrary number of env evaluations
#####################################################################

evaluation_number = 0  # evaluation counter
total_time_limit = 8 * 60 * 60
global_time_start = time.time()




local_env = RailEnv(
    width=0,
    height=0,
    rail_generator=rail_from_file("../../scratch/test-envs/Test_" + str(args.level) + "/Level_0.pkl"),
    number_of_agents=args.agents,
)

observation, info = local_env.reset()

number_of_agents = len(local_env.agents)

max_time_steps = int(4 * 2 * (local_env.width + local_env.height + 20))


    #####################################################################
    # step loop information
    #####################################################################

time_taken_by_controller = []
time_taken_per_step = []




time_init = time.time()

debug = False
render = True

CBS = PythonCBS(local_env, framework, time_limit, default_group_size, debug, replan,stop_threshold,agent_priority_strategy,neighbor_generation_strategy)
CBS.search(agent_percentage, max_iteration)
CBS.buildMCP()

if render: 
    env_renderer = RenderTool(local_env, screen_width=2000, screen_height=2000, show_debug=True)

    env_renderer.render_env(show=True, frames=False, show_observations=False, show_predictions=False, show_rowcols=True)



for steps in range(5000):

    if steps >= 1:
        prev_done = copy.deepcopy(done)

    action = CBS.getActions(local_env, steps, 1.0)
    _, all_rewards, done, info = local_env.step(action)

    if render:
        env_renderer.render_env(show=True, frames=False, show_observations=False, show_predictions=False)
        # time.sleep(0.2)
        input("Weiter?")


    if done['__all__']:

        if sum(list(all_rewards.values())) < 0:

            print(sum(prev_done.values()))

        else:
            print(int(sum(list(all_rewards.values()))))

        print(f"Steps: {steps}")
        print(f"Time: {time.time()-time_init:5f}")
        break
