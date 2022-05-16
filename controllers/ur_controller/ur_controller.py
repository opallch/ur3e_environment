"""factory controller."""

from coco import generate_nodes, get_node_field
from composer import Composer
from writers import writers

import os
if os.getenv('ROS_DISTRO') is not None:
    from webots_ros2_driver_webots.controller import Supervisor
else:
    from controller import Supervisor

supervisor = Supervisor()
timestep = int(supervisor.getBasicTimeStep())

# Generate nodes
generate_nodes(supervisor, 'spec.yml', warmup=5, export=f'scene.yml')

# Example: generate language descriptions for chairs
composer = Composer(writers)
i = 0
while True:
    chair = supervisor.getFromDef(f'CHAIR({i})')
    if chair is None:
        break
    r,  g, b = get_node_field(chair, 'color')
    print(composer({'color': [int(r*255), int(g*255), int(b*255)]}))
    i +=  1