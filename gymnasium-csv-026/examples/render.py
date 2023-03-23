#!/usr/bin/env python

import gymnasium as gym
import gymnasium_csv

env = gym.make('gymnasium_csv-v0',
               render_mode='human',  # "human", "text", None
               inFileStr='../assets/map1.csv',
               initX=2,
               initY=2,
               goalX=7,
               goalY=2)
env.reset()
env.render()
