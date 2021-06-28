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

import os
import subprocess
import numpy as np
import time

#####################################################################
# parameter trigger local testing and remote testing
#####################################################################

remote_test = False
save_txt_file = False
debug_print = False
env_renderer_enable = False
input_pause_renderer = False

#####################################################################
# local testing parameters
#####################################################################
x_dim = 30
y_dim = 30

# parameters to sparse_rail_genertor
max_num_stations = 3
given_seed = 12
given_max_rails_between_cities = 2 # not clear what this does
given_max_rails_in_city = 3 # not clear what this does
given_num_agents = 10

# speed profile, 1 -> speed is 1, 1_2 -> speed 0.5, 1_3 -> speed 1/3, etc. sum has to be 1
given_1_speed_train_percentage = 1
given_1_2_speed_train_percentage = 0
given_1_3_speed_train_percentage = 0
given_1_4_speed_train_percentage = 0

# malfunction parameters
malfunction_rate = 0.4          # fraction number, probability of having a stop.
min_duration = 3
max_duration = 20


agent_percentages = [1.1] * 400  # agent percentages for initial planning, learnt from local instances
replan = [(i % 10 != 0) and (10 <= i < 290) for i in range(400)]  # replan or not


max_iterations = [100]  # max iterations for LNS, learnt from local instances

frameworks = ["LNS"] * 40
for i in range(len(max_iterations)):
    if max_iterations[i] > 0:
        frameworks[i] = "Parallel-LNS"



#####################################################################
# Instantiate a Remote Client
#####################################################################
if remote_test:
    remote_client = FlatlandRemoteClient()


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
    rail_generator=rail_from_file("../../scratch/test-envs/Test_8/Level_0.pkl"),
    number_of_agents=13,
)

observation, info = local_env.reset()

number_of_agents = len(local_env.agents)

max_time_steps = int(4 * 2 * (local_env.width + local_env.height + 20))


    #####################################################################
    # step loop information
    #####################################################################

time_taken_by_controller = []
time_taken_per_step = []
steps = 0



debug = False
time_limit = 200
default_group_size = 5  # max number of agents in a group
stop_threshold = 30
agent_priority_strategy = 3
neighbor_generation_strategy = 3
CBS = PythonCBS(local_env, frameworks[evaluation_number//10], time_limit, default_group_size, debug, replan[evaluation_number],stop_threshold,agent_priority_strategy,neighbor_generation_strategy)
CBS.search(agent_percentages[evaluation_number], max_iterations[evaluation_number//10])
CBS.buildMCP()


while True:

    action = CBS.getActions(local_env, steps, 3.0)
    _, all_rewards, done, info = local_env.step(action)

    steps += 1

    if done['__all__']:
        print("Reward : ", sum(list(all_rewards.values())))
        print("all arrived after", steps)
        CBS.clearMCP()
        
        break



print("Evaluation of all environments complete...")
