#!/usr/bin/env python3

import sys
sys.path.append('../')

from drone_env_viz.viz_lib._drone_viz import AgentVis

if __name__=="__main__":
    agent = AgentVis(agent_name="dude")
    agent.run()