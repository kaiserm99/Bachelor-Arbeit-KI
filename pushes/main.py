#!/usr/bin/env python3

"""
Copyright 2021, University of Freiburg
Bachelorarbeit - Foundations of Artificial Intelligence

Author: Marco Kaiser <kaiserm@informatik.uni-freiburg.de>

Description:

Usage:

"""

import time

from flatland.core.env import Environment
from flatland.envs.rail_env import RailEnv
from flatland.envs.rail_generators import sparse_rail_generator
from flatland.envs.schedule_generators import sparse_schedule_generator
from flatland.utils.rendertools import RenderTool

import src.mapf as mapf

direction_to_str = {-1 : "pass...", 0: "North", 1: "East", 2: "South", 3: "West"}
action_to_str = {-1 : "pass...", 0 : "Do nothing", 1: "Left", 2 : "Forward", 3 : "Right", 4 : "Stop moving"}


render = True
print_more_info = False

try:

    time_start = time.time()

    # Initalize the Environment
    rail_generator=sparse_rail_generator(
        max_num_cities = 5,
        grid_mode= False,
        max_rails_between_cities = 5,
        max_rails_in_city = 5, 
        seed=0
    )
    
    env = RailEnv(
        width=50, height=50,
        rail_generator=rail_generator,
        schedule_generator=sparse_schedule_generator(),
        number_of_agents=17
    )

    obs, info = env.reset()
    print(f"Created an reseted the Environment in {time.time()-time_start:5f}sec\n")
    
    
    # Just do one dummy step so the Trains get a Position. My assumption is, that it is always the same
    env.step({i : 2 for i in range(env.get_num_agents())})
    
    # Check if the result of the prioritized planning is valid
    res = mapf.pp(env)
    if res == -1: raise Stop

    # For rendering the Environment and the steps done by the agents
    if render: 
        env_renderer = RenderTool(env, screen_width=2000, screen_height=2000)
        env_renderer.render_env(show=True, frames=False, show_observations=False, show_predictions=False)
    
    # Empty action dictionary which has the predicted actions in it for each step
    action_dict = dict()
    
    # For Loop with all the steps predicted by the agent
    for step in range(2000):
        
        for handle in range(env.get_num_agents()):
        
            actions = res[handle]
            
            if not actions.is_empty() and info['action_required'][handle]:
                action, pred_positon = actions.pop(0)
                action_dict[handle] = action
                
                env.dev_pred_dict[handle] = actions.positions
            else:
                action_dict[handle] = 0
        
        # Do the actual step in the Environment based on the action_dict computed previously 
        obs, all_rewards, done, info = env.step(action_dict)
        
        # Print the current status of the agents in each iteration
        print(f"[{step+1:3}] In goal: {[handle for handle, status in done.items() if status]}")
        
        if print_more_info:
            print("Iteration:", step)
            for handle, action in action_dict.items():
                print(f"<{handle}> Action: |{action}|, Position: {env.agents[handle].position}, Target: {env.agents[handle].target}, Direction: {direction_to_str[env.agents[handle].direction]}")

        if render: 
            env_renderer.render_env(show=True, frames=False, show_observations=False, show_predictions=True)
            time.sleep(0.1)

        if done["__all__"]:
            print(f"\nAll Agents are in their targets! After {step+1} iterations.")
            break
            
finally:
    if render : env_renderer.close_window()