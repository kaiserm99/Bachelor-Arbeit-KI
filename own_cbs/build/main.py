#!/usr/bin/env python3

import time

from flatland.envs.rail_env import RailEnv
from flatland.envs.rail_generators import rail_from_file
import numpy as np

import libFlatlandCBS as FlatlandCBS

from flatland.utils.rendertools import RenderTool, AgentRenderVariant


def set_speed(env, speed_ration_map):

    for a in env.agents:
        a.speed_data['speed'] = np.random.choice(list(speed_ration_map.keys()), p=list(speed_ration_map.values()))



speed_ration_map = {1. / 1.: 0.,
                    1. / 2.: 0.,
                    1. / 3.: 1.,
                    1. / 4.: 0.}

time_start = time.time()
env = RailEnv(
    width=0,
    height=0,
    rail_generator=rail_from_file("../../scratch/test-envs/Test_6/Level_0.pkl"),
    number_of_agents=2
)


_, info = env.reset()
# set_speed(env, speed_ration_map)
_,_,_,_ = env.step({ 0 : 2, 1 : 2})




cbs = FlatlandCBS.FlatlandCBS(env)
print(f"Created an reseted the Environment in {time.time()-time_start:5f}sec\n")

time_start = time.time()
status = FlatlandCBS.mainSearch(cbs)

if status == 0:
    print(f"\nFound a solution in {time.time()-time_start:5f}sec\n")
    print(cbs.getStatistic())

else:
    exit(1)


print(FlatlandCBS.getActions(cbs))




"""
render = True
    

try:

    if render: 
        env_renderer = RenderTool(env, screen_width=2000, screen_height=2000, show_debug=True)

        env_renderer.render_env(show=True, frames=False, show_observations=False, show_predictions=False, show_rowcols=True)


    for step, action in enumerate([2, 2, 3, 1, 2, 2, 2, 1, 3, 2, 2, 1, 2, 2, 1, 2, 2, 3, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 2, 2, 2, 2, 2, 1]):

        obs, all_rewards, done, info = env.step({0 : action})
        print(f"[{step:3}] In goal: {[handle for handle, status in done.items() if status]}")
        print(action)

        if render:
            env_renderer.render_env(show=True, frames=False, show_observations=False, show_predictions=False)
            time.sleep(0.5)

finally:
    if render : env_renderer.close_window()
"""

