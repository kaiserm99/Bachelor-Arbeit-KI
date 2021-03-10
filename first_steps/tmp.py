import PIL

import numpy as np
import time
from flatland.envs.rail_generators import complex_rail_generator
from flatland.envs.schedule_generators import complex_schedule_generator
from flatland.envs.rail_env import RailEnv
from flatland.utils.rendertools import RenderTool
from flatland.envs.rail_generators import sparse_rail_generator
from flatland.envs.rail_generators import random_rail_generator
from flatland.envs.rail_generators import rail_from_manual_specifications_generator
from flatland.envs.observations import TreeObsForRailEnv


"""
sparse_env = RailEnv(
    width=30,
    height=30,
    rail_generator=sparse_rail_generator(
        max_num_cities=5,  # Number of cities (= train stations)
        grid_mode=False,  # Distribute the cities evenly in a grid
        max_rails_between_cities=2,  # Max number of rails connecting to a city
        max_rails_in_city=3  # Number of parallel tracks in cities
    ),
    number_of_agents=10
)

observation, info = sparse_env.reset()


def render_env(env):
    env_renderer = RenderTool(env, gl="PILSVG")
    env_renderer.render_env(show=True, show_predictions=False, show_observations=False)

    image = env_renderer.get_image()
    pil_image = PIL.Image.fromarray(image)

    pil_image.show()"""



# render_env(sparse_env)

class SimpleObs(ObservationBuilder):
    """
    Simplest observation builder. The object returns observation vectors with 5 identical components,
    all equal to the ID of the respective agent.
    """

    def reset(self):
        return

    def get(self, handle):
        observation = handle * np.ones(5)
        return observation


env = RailEnv(width=7, height=7,
              rail_generator=random_rail_generator(),
              number_of_agents=3,
              obs_builder_object=SimpleObs())
env.reset()
