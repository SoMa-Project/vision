# For the cylindrical section, the size vector reads:
# [inner_radius_along_x, radius_width, angle, height_along_z]
# For the spherical sector:
# [inner_radius_along_x, radius_width, angle]
# For the box:
# [width_along_x, length_along_y, height_along_z]
int32 CYLINDER = 0
int32 CYLINDRICAL_SECTOR = 1
int32 SPHERE = 2
int32 SPHERICAL_SECTOR = 3
int32 BOX = 4

int32 type
geometry_msgs/PoseStamped pose
geometry_msgs/PoseStamped center
float64[] size
float64[] image_size
