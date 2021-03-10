import getopt
import random
import sys
import time
from typing import List

import numpy as np

from flatland.core.env_observation_builder import ObservationBuilder
from flatland.core.grid.grid4_utils import get_new_position
from flatland.envs.rail_env import RailEnv
from flatland.envs.rail_generators import complex_rail_generator
from flatland.envs.schedule_generators import complex_schedule_generator
from flatland.utils.misc import str2bool
from flatland.utils.rendertools import RenderTool

random.seed(100)
np.random.seed(100)


class SingleAgentNavigationObs(ObservationBuilder):
    """
    We build a representation vector with 3 binary components, indicating which of the 3 available directions
    for each agent (Left, Forward, Right) lead to the shortest path to its target.
    E.g., if taking the Left branch (if available) is the shortest route to the agent's target, the observation vector
    will be [1, 0, 0].
    """

    def __init__(self):
        super().__init__()

    def reset(self):
        pass

    def get(self, handle: int = 0) -> List[int]:
        agent = self.env.agents[handle]

        if agent.position:
            possible_transitions = self.env.rail.get_transitions(*agent.position, agent.direction)
        else:
            possible_transitions = self.env.rail.get_transitions(*agent.initial_position, agent.direction)

        num_transitions = np.count_nonzero(possible_transitions)

        # Start from the current orientation, and see which transitions are available;
        # organize them as [left, forward, right], relative to the current orientation
        # If only one transition is possible, the forward branch is aligned with it.
        if num_transitions == 1:
            observation = [0, 1, 0]
        else:
            min_distances = []
            for direction in [(agent.direction + i) % 4 for i in range(-1, 2)]:
                if possible_transitions[direction]:
                    new_position = get_new_position(agent.position, direction)
                    min_distances.append(
                        self.env.distance_map.get()[handle, new_position[0], new_position[1], direction])
                else:
                    min_distances.append(np.inf)

            observation = [0, 0, 0]
            observation[np.argmin(min_distances)] = 1

        # self.env.dev_obs_dict[handle] = {(9.0, 8.0), (9.0, 7.0), (9.0, 6.0), (9.0, 5.0), (9.0, 4.0), (9.0, 3.0), (9.0, 2.0), (9.0, 1.0), (9.0, 0.0), (8.0, 0.0)}
        # self.env.dev_pred_dict[handle] = {(9.0, 8.0), (9.0, 7.0), (9.0, 6.0), (9.0, 5.0), (9.0, 4.0), (9.0, 3.0), (9.0, 2.0), (9.0, 1.0), (9.0, 0.0), (8.0, 0.0)}

        return observation


def main():


    # Create the Rail Env and specify all the needed parameters
    env = RailEnv(width=15, height=15,
                  rail_generator=complex_rail_generator(nr_start_goal=10, nr_extra=1, min_dist=5, max_dist=99999,
                                                        seed=1), schedule_generator=complex_schedule_generator(),
                  number_of_agents=1, obs_builder_object=SingleAgentNavigationObs())

    obs, info = env.reset()

    # Just to render (print) the Env in a Picture and see the agent move
    env_renderer = RenderTool(env, screen_height=2000, screen_width=2000)

    env_renderer.render_env(show=True, frames=True, show_observations=False)


    # Main loop to step in the current Env by an Agent based on the given observation 
    for step in range(100):

        # Obs is a dictionary with all the observations of the agents, calculate the next action
        action = np.argmax(obs[0]) + 1

        obs, all_rewards, done, _ = env.step({0: action})
        print("Rewards: ", all_rewards, "  [done=", done, "]")
        env_renderer.render_env(show=True, frames=True, show_observations=False, show_predictions=True)
        
        time.sleep(.1)
        if done["__all__"]:
            break
    env_renderer.close_window()


if __name__ == '__main__':
    main()
