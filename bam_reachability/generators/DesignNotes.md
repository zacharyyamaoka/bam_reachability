# the issue is that if you have joint limits alot of this will be out of range...
Recntgualr generator
- Width extent
    - Simple grid generator


rectangle_point_generator(scale=(0.1,0.1,0.1), step=0.1)

- Spherical Generator

sphereical_generator(diameter(1), step=0.1, sign=-1)
    points = rectangle_point_generator(scale=(diameter, diameter, diameter), step=0.1)
    inside_points = mask_sphere(points)
    return inside points

Donute_generator(inner_diameter=(0.1), out_diamtere=(1), step=0.1)
    points = rectangle_point_generator(scale=(out_diamtere, out_diamtere, out_diamtere), step=step)

    inside_points = mask_sphere(points)
    inside_outside_points = mask_sphere(inside_points, sign=-1)

    return (inside_outside_points)


def mask_sphere(points, sign=1)


Use a recntgualr generator and then mask the points that layoutside of the sphere

- Donute generator
Use spherical generator twice (inside or outside!)

FKGenerator
- generate valid frames that robot can reach
- Avoid wasteing time on areas outside of robots workspace define by joint limits...
- This could be helpful just to overlay, and get a quick idea

- Discreteize J1, J2, J3, and sample points (should be a sphere that respects the joint angles)
- Treat as pointcloud and voxelize to align to grid (you can set the discrtization option)
- Add full orietation to each point (you don't know where base_link is going to be head of time...)
- If you do then you can set the orientation and a distance from it of Pi for example...


Pose generator, give a random orientation (in this case can be z axis of base_link), but you can also specify another one, and a range to sample from
(hemisphere sampler...)

Mesh generator():
Implement to achieve similar funcaiolity to Reach2

Either pase in a list of poses (6 dof) or 3 dof and orientation list (which is used for each pose) If you only pass in 3DOF then it assumes, and it needs a seperate list for the orientations.