#!/usr/bin/env python
# coding: utf-8
# visualize maps using Ctrl key to switch


import sys
carla_folder = '/home/d/software/carla/CARLA_0.9.15'
sys.path.append(carla_folder + '/PythonAPI/carla/agents')
sys.path.append(carla_folder + '/PythonAPI/carla')

import carla

town_names = ['Town01','Town02','Town03','Town04','Town05','Town06','Town07','Town10HD',]

def read_map(town_name):
    prefix = '/home/d/software/carla/CARLA_0.9.15/CarlaUE4/Content/Carla/Maps/OpenDrive/'
    file = prefix + town_name + '.xodr'
    with open(file, 'r') as fp:
        return carla.Map(town_name, fp.read())


# In[6]:


import xml.etree.ElementTree as ET
file = 'srunner/examples/SignalizedJunctionRightTurn.xml'
file = 'srunner/examples/SignalizedJunctionLeftTurn.xml'
file = 'srunner/examples/FollowLeadingVehicle.xml' # maybe has problems???
file = 'srunner/examples/ChangeLane.xml'
file = 'srunner/examples/VehicleTurning.xml'
file = 'srunner/examples/NoSignalJunction.xml'
# file = 'srunner/examples/FreeRide.xml' # remove this???
file = 'srunner/examples/RunningRedLight.xml'
file = 'srunner/examples/LeadingVehicle.xml' # don't use this???
file = 'srunner/examples/OppositeDirection.xml'
file = 'srunner/examples/CutIn.xml'
file = 'srunner/examples/ObjectCrossing.xml'
file = 'srunner/examples/ControlLoss.xml'
tree = ET.parse(file)


# In[7]:


def scenario_gen():
    attr_names = ['x', 'y', 'z', 'yaw', 'pitch', 'roll']
    scenarios = next(tree.iter("scenarios"))
    for i, scenario in enumerate(scenarios.iter('scenario')):
        other_vehicles = []
        for vehicle in scenario.iter("other_actor"):
            other_vehicles.append([float(vehicle.get(attr_name, 0.)) for attr_name in attr_names])

        ego = next(scenario.iter("ego_vehicle"))
        ego_value = [float(ego.get(attr_name, 0.)) for attr_name in attr_names]
        yield (scenario, ego_value, other_vehicles)


def next_data(gen):
    scenario, ego_value, other_vehicles = next(gen)
    town_name = scenario.attrib['town']
    m = read_map(town_name)
    waypoints = m.generate_waypoints(distance=1.0)

    ego_loc = carla.Location(ego_value[0], ego_value[1], ego_value[2])
    ego_wp = m.get_waypoint(ego_loc)

    other_wp = []
    for v in other_vehicles:
        loc = carla.Location(v[0], v[1], v[2])
        other_wp.append(m.get_waypoint(loc))
    return (scenario, m, waypoints, ego_wp, other_wp)


# In[8]:


# get_ipython().run_line_magic('matplotlib', 'tk')
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from srunner.tests.carla_mocks.agents.navigation.global_route_planner import GlobalRoutePlanner
from environment.create_scenario_waypoints import get_waypoint_from_scenario

''' Press any key to show to the next scenario '''

gen = scenario_gen()


def onpress(event):
    ix, iy = event.xdata, event.ydata

    try:
        scenario, m, waypoints, ego_wp, other_wp = next_data(gen)
    except StopIteration:
        plt.close()
        return
    
    end_wp = get_waypoint_from_scenario(scenario.attrib['name'], ego_wp)

    plt.cla()
    # plt.clf()

    ax = plt.gca()

    x = [waypoint.transform.location.x for waypoint in waypoints]
    y = [waypoint.transform.location.y for waypoint in waypoints]
    ax.scatter(x, y, s=.01, marker='.', c='b', label='map')

    for l in m.get_all_landmarks():
        l_wp = m.get_waypoint(l.transform.location, project_to_road=True)
        # l_wp = l
        if l.name.lower().endswith('stop'):
            ax.scatter(l_wp.transform.location.x, l_wp.transform.location.y, s=.1, marker='H', c='r', label='stop')
        elif l.name == 'Signal_3Light_Post01':
            ax.scatter(l_wp.transform.location.x, l_wp.transform.location.y, s=.1, marker='s', c='y', label='traffic lights')

    ax.scatter(ego_wp.transform.location.x, ego_wp.transform.location.y, s=.1, marker='x', c='k', label='vehicle')
    if end_wp is not None:
        ax.scatter(end_wp.transform.location.x, end_wp.transform.location.y, s=.1, marker='o', c='k', label='vehicle')

        route = GlobalRoutePlanner(m, 10).trace_route(ego_wp.transform.location, end_wp.transform.location)
        for wp, _ in route:
            ax.scatter(wp.transform.location.x, wp.transform.location.y, s=.1, marker='o', c='k', label='vehicle')

    for wp in other_wp:
        ax.scatter(wp.transform.location.x, wp.transform.location.y, s=.1, marker='x', c='m', label='vehicle')

    plt.title(scenario.attrib['name'])

    ax.axis("equal")
    ax.invert_xaxis()  # Invert the y-axis

    plt.axis('off')
    plt.show()

fig = plt.figure(figsize=(16, 9), dpi=300)

# cid = fig.canvas.mpl_connect('button_press_event', onclick)
cid = fig.canvas.mpl_connect('key_press_event', onpress)

plt.show()


# In[ ]:




