import numpy as np


# Load Maps into workspace

# View all Points inside of workspace

# go through all the workspaces

# View all the points

# View the bounding convex hull

# sample some points and send the robot there...

dex_map = ReachabilityMap.load("/home/bam/python_ws/bam_reachability/maps/ur/ur5e/ur5e_table_1.2x1.2x1.0_0.10_90x45x360_15_jul_2025.pkl")
hemi_map = ReachabilityMap.load("/home/bam/python_ws/bam_reachability/maps/ur/ur5e/ur5e_table_1.2x1.2x1.0_0.10_85x85x360_15_jul_2025.pkl")


np.random.seed(42)
workspace = ReachabilityWorkspace(dex_map, hemi_map)

ok fine... it does seem simpler to just mask and calculate the various succeses...

# One thing I don't like is how the maps are redundant!

workspace.dexterous.sample(n=10, replace=True) # just like numpy sampling...


workspace.dexterous.to_pcd()

