

# Installation

Activate desired venv, then:

python3 -m pip install -e ~/python_ws/bam_reachability


# Work Space Analysis

References: 

[REACH ROSCon 2019 Macau Talk](https://vimeo.com/378683038)
- This is for workpiece, not as relevant

[Simple Reachability](https://github.com/vonunwerth/simple-reachability)
- Uses moveit as a plugin, good idea tbh..

[Reuleaux](https://rosindustrial.org/news/2016/10/18/google-summer-of-code-project-workspace-analysis-and-base-placement#:~:text=The%20first%20project%20goal%20was,and%20reachability%20of%20that%20spheres)
- Great ideas for Viz

http://wiki.ros.org/reuleaux

[pycapacity](https://github.com/auctus-team/pycapacity)
This is great at some point, but I am looking for something simpler for now!


[Compas Fab Reachability](https://compas.dev/compas_fab/latest/examples/07_reachability_map/01_reachability_map.html)

https://github.com/compas-dev/compas_fab/blob/main/src/compas_fab/robots/reachability_map/reachability_map.py


- It is faster to just compute it for all the orientations once, relative to the robot base_frame and then keep thoose points




Ok what is the fastest way for me to do this?
- This is actually a fairly simple piece of code.
- I already have the IK/FK
- I already have the ROS Viz capabilities
- There is just a bit of logic for the Pose Generator and the vizulaiser for how to view the frames.
- It does feel like I can reference some things and quickly put together a python script
- Important for it to eb pure python

Its a very nice API!


Ok here is the functionality I want.

Create a ReachabilityMap,
- List of points (in the order they were tested)
    - Each point knows how many IK solutions where tried
    - How many IK solutions where succesful
    - A list is kept of each individual result, ands its order
    - If the IK/FK was consistent [OPTIONAL]


Pose Generator
- The order of the frames should be tagged/keep constant so its easy to afterwards
select the succesful ones within a certain hemisphere
- This is cool beacuse you can later use it for workpiece analysis etc...
- Generates the poses that the 
    - params: sample density
    

Kinematic Function
- IK, takes in a pose, and returns if succeful or now
- Could be moveit, or pyreach, etc.
- Params:
    - check_collision: true, false
    - solver: kdl, cf

RosVizualizer

- Ability to set slice direction, and iterate through by sending enter in CLI
- Ability to slice based on directions of base_frame 
- Ability to only include 
- Be able to define a recntagular taskspace by a center pose, and extents
- See all pointswithins within there that meet the criteria

- Params:
    - Number of buckets (you can discreize them)
    - Color Map
    - size - 
    - Alpha
    - Density (you can further voxelize?)
    - hemisphere
    -


- Ability to set sample density
- Ability to verify IK/FK consistenty in workspace

Key things I would like to understand

2. Workspace/Reach
- What is the maximum workspace that I can reach (give me an idea of where I could drop things off in bins)
- What is the dexterous workspace, with 6 DOF range of motion? (give me an idea of where I can turn an object to view the camera)
- What is the hemisphere workspace? (give me an idea of where I can do complete pick and place on a table)
- What is the 4 DOF workspace? (keep arm point vertical from table)
- What are the rectangular workspaces? (width and height)

- You can use forward kinematics to get an idea of the bounds
- You can roughly sample the IK to generate the full mesh grid. Then you can generate the workspaces by seeing if they match the criteria (add to list)
- Then you can generate the shapes using algorithims for point inclusion. (that fits inside a points)
- Then return all the point lists, and the shapes
- Then create a viz algroihtim that steps thorugh slice by slice on the point relative to the table.
- I can turn on the denisty parameter
- I can turn on/off whether to verify the IK/FK
