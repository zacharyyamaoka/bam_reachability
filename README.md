# bam_reachability
---

## Reasons for this package

Kinematics is likely the most important package for a robotic start up using kinematics chains (arm, legs, etc.)

Its definetly a basic function, there is not abstractions underneath it. its cold hard math.

This package is meant to help analyze and verify the correctness of kinematics.

---

1. I was getting frusterated when making calls to MoveIt, that planning would fail beacuse the points where outside the workspace. 
2. During testing, it was not simple to generate random points within the workspace of a robot.

My first solution was to use the Moveit Rviz Marker to manually move the robot to positions and save the location:

```python
self.HOME_JOINT_STATE = get_joint_state(self.joint_names, [-0.003752, 0.027098, -1.432066, -1.768329, -1.633024, 1.568815])
self.HOME_POSE = get_pose([0.290701, -0.062931, 0.452099], [1.570788, -0.031643, -0.066011])
self.HOME_POSE_STAMPED = pose_to_pose_stamped(self.HOME_POSE, self.base_link)

self.READY_JOINT_STATE = get_joint_state(self.joint_names, [0.002404, 1.384432, -1.882157, -2.236169, -1.078447, 1.602683])
self.READY_POSE = get_pose([0.292174, -0.053916, 0.090194], [1.804073, 0.356935, 0.531201])
self.READY_POSE_STAMPED = pose_to_pose_stamped(self.READY_POSE, self.base_link)
```

I have this script: /home/bam/bam_ws/src/bam_misc/bam_calibration/scripts/save_curr_pose.py that auto generates this code,
and the matching code for the robot.srdf

This is great for being very specific about points, and at somepoint something like this could be used to program drop points for the robot (either virtually or IRL)

The script remains the same which is saving the current pose, all that changes is, do you moveit via moveit? or via gravity compensation back driving (like eva)?


Limitaions of this approach though are: 

1. You need to update it if the robot description changes
2. Not practical for generating a large number of random poses
3. No notion of dexterous/hemisphere workspace

An interesting opporunity:

- If you are discrtizing the workspace (heatmap), then you can actaully precompute, all the possible paths ahead of time to make sure they are all succesful!
- This may take a while... so you could start with a lower resolution version

## Pose Representation

Options:

1. xyz rpy (euluer)
2. xyz xyzw (quaternion)
3. xyz R
4. T


Simple, Fast, Clear.
- using Transform and Rotation matrices is very specific and natural way
- Split position and orietation so that you can mix and match
- When calculating targets they are often in matrices beacuse you need to find say the position of an object in a camera frame relative to the base frame
- The IK solutions return matrices that I am converting...
- The generators return matrices
- When comparing you cannot do with rpy, beacuse not unique

```
    def fk(self, q: list) -> PoseStamped:

        # TODO check if q is within joint limits...
        q_reflect = self.reflect(q) # careful not to override q

        T = forward_kinematics_offset_wrist(q_reflect, self.dh_params)

        pose = matrix_to_pose_stamped(T, self.base_link)

        if self.use_tool0:
            # Transform pose from ee_link to tool0 
            pose = apply_offset(pose, rpy=[-np.pi/2, 0, -np.pi/2], local=True)

        return pose
```

- Matrices don't depend on any message format like PoseStamped, etc..

Ok confirmed! I will represent as (xyz and R)

I do think there is great power in just simple python packages which describe exactly what they are doing... 

# TODO

- The most important thing is moveit, as that is what is actually is called
- If I want to use the other packages though freely I should also verify they are correct!

- I initally want to develop this to be a purely standalone python library.
- I am now changing pace a bit and I will make it use ROS msgs types
- The disadvantage is that now it cannot really standalone. 
- The adventage is that this package is actually quite integral! and its better to use the same languague!

- FML I couldn't do it! As I started convert to ROS, I couldn't bring my self to make this beutiful pure python package be filled with these ROS utils...

Ok I think its very reasonable that the kin_wrapper works with numpy arrays. This makes it indepent from ROS msgs, which is definetly a bonus! 
As most of my implementations are in ROS. I am ok for simpilicty to have it a ros package for now...

why not make it external though? yes lets do it!

The generator functions are very useful for discretizing action spaces. 


your_kinematics -> kin_wrapper -> reachability_map -> analysis/visualization

One down side is that if you want to really understand moveit, you will likely need to more directly call the kinematics, as via ros is way to slow...
Potetially could be done via c++ to just generate the data? or could use moveit python direct api, lots of options...

What representation to use for orientations and positions, etc?

Euler, Quaternion, Rotation matrix? I think there are trades offs between all of them... I can just continue as I am for now, fairly easy to change in the future if needed...

Todo

Kinematics
- [X] Verify Mock Kinematics
    Learnings:
        - Nice to have control over when it succeeds and fails
- [X] Verify Moveit Kinematics
    Learnings:
        - RCLPY is very slow, after around 500 calls hits 100% CPU usage (check with HTOP), and really slows down
        - Making lots of calls with rclpy.spin_until_future_complete() doesn't seem like a good idea.
        - Adding wait timers, etc, I couldn't improve it
        - It's an open issue: 
- [X] Verify Offset Wrist Kinematics
    Learnings:
        - Inconsitent IK/FK due to numerical instability it seems when poses are too perfectly aligned
        - Making hemisphere 89 deg, using random poses, etc solves the problem
        - I do think its ok to fail and understand why it fails, I think its also important to achieve a state with no errors, thats a special state
- [X] Verify Pinnochio Kinematics
    Learnings:
        - Nice benefit of modular IK/FK is that I can provide the FK from pin and IK from Offset wrist

Analysis 
- [X] Run full analysis for Mock Robot
- [X] Run full + table analysis for UR robot
    Learnings:
        - Table analysis is really, really awesome and powerful. I like how I can completely were there are solutions now etc
        - I don't think I actually need to do anything more here... if I load up the table transform, then I can just analyze it
          seperatly in mesh cat, and walk through all the different poses, etc.. I think this is much more powerful..
- [ ] Run full + table analysis for BAM robot

Bounds
- [ ] Create function that finds the smallest/largest primative shape that contains/is within a pointcloud
- [ ] Create functions to sample points within full or table dexterous, 4 dof or hemisphere workspace
- [ ] Create function that reports error if pose is outside dexterous, 4 dof, hemisphere or reach workspace

Foxglove Viz
- [ ] Publish orietanted cube marker for table workspace
- [ ] Publish full dexterous + reach workspace mesh markers
- [ ] Calculate dexterous, reach, 4 dof, hemisphere workspace inside table workspace (Union)
- [ ] Publish point cloud for negative 4 dof and hemisphere points inside table workspace

Publish dexterous, reach, 4 DOF, Hemisphere workspace mesh markers 

- Run Tests on each robot
- Save result
- Load result
- Vizualise results for
    - reach at least one point
    - reach in between...
    - reach with all points
    - Given a z direction
        - for hemisphere and 4 dof workspace

- Save Meshes for, reach and dexterous, 
- Save meshes for hemisphere, and 4 Dof workspace (given a z direction, by default is baselink z direction)
- Save negative pointcloud for poitns that are outside of the meshes

- Create Table generator (this is important for discreizating an action space...)
- Ability to save TF between baselink and "table_Frame" to human editable YAML
- Ability to set height, and +x,-x, +y,-y extents (assuming that Z is point upwards)
- Ability to run a script that will then generate points inside of that grid 
    - Run result agian, and then generate 
    - Another option would be to mask the full workspace... both are about similar, this one is cool beacuse you get poitns aligned with workspcae though

- Create ros node that at start up publishes the meshes and pointclouds, so that they can be easily view.
- just comment out the lines to turn of publishing of certain things if you want. 

- What is the maximum workspace that I can reach (give me an idea of where I could drop things off in bins)
- What is the dexterous workspace, with 6 DOF range of motion? (give me an idea of where I can turn an object to view the camera)
- What is the hemisphere workspace? (give me an idea of where I can do complete pick and place on a table)
- What is the 4 DOF workspace? (keep arm point vertical from table)
- What are the rectangular workspaces? (width and height)

# Installation

Pinnochio has to be installed seperatetly
https://github.com/stack-of-tasks/pinocchio?tab=readme-ov-file
https://stack-of-tasks.github.io/pinocchio/download.html

OLD, now installing as a Ros package

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


[Pinnochio Workspace Example](https://github.com/stack-of-tasks/pinocchio/blob/devel/examples/reachable-workspace-with-collisions.py)

- It is faster to just compute it for all the orientations once, relative to the robot base_frame and then keep thoose points



- Create a ROS2 node that at start up
- Publsihes the workspaces etc. for the points...

I could use shapes.. or I could use convex hull meshes...
https://github.com/auctus-team/pycapacity

Ok that actually would work and foxglove has a wayt to vizualise it with opacity....

I would need to generate the workspace, export the stls.. and then I can publish them. another option would be to send it live..

Let me do a quick test to see if I can viz a convex hull in rviz?

yes....

### Notes

- I am happy with the speed it took to develop this. By copying pasting alot of the code, and using it as base line. Its like having
a really great seed for the trial and error optimization. It saves so much time! No need to reinveint the wheel. Its this nice balance...


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
