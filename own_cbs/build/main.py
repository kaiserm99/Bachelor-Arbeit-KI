#!/usr/bin/env python3

import time

from flatland.envs.rail_env import RailEnv
from flatland.envs.rail_generators import rail_from_file

import libFlatlandCBS as FlatlandCBS


time_start = time.time()
env = RailEnv(
    width=0,
    height=0,
    rail_generator=rail_from_file("../../scratch/test-envs/Test_8/Level_0.pkl"),
    number_of_agents=13
)


_, info = env.reset()

cbs = FlatlandCBS.FlatlandCBS(env)
print(f"Created an reseted the Environment in {time.time()-time_start:5f}sec\n")


FlatlandCBS.mainSearch(cbs)
