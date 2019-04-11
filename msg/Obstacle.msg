#This message is used to define an obstacle characteristics

#uint8 id1
#uint8 id2

#float32[] centroid

uint8 type1
uint8 type2

geometry_msgs/Point location1
geometry_msgs/Point location2

#types
uint8 WALL=0
uint8 RAMP=1
uint8 BOX=2
uint8 TUNNEL_BOX=3
uint8 SPHERE=4
uint8 CYLINDER=5
uint8 TUNNEL_CYLINDER=6
uint8 CONE=7
uint8 SMALL_TUNNEL_ENTRANCE=8
uint8 LARGE_TUNNEL_ENTRANCE=9
uint8 RAMP_ENTRANCE=10
uint8 SMALL_TUNNEL_EXIT=11
uint8 LARGE_TUNNEL_EXIT=12
uint8 RAMP_EXIT=13
uint8 NONE=14


#float32[] dimensions
#index on how to access dimensions for different shapes

uint8 WALL_X=0
uint8 WALL_Y=1

uint8 RAMP_X=0
uint8 RAMP_Y=1
uint8 RAMP_Z=2
uint8 RAMP_C=3

#for BOX and TUNNEL_BOX
uint8 BOX_X=0
uint8 BOX_Y=1
uint8 BOX_Z=2

uint8 SPHERE_RADIUS=0

#for CYLINDER and TUNNEL_CYLINDER but TUNNEL_CYLINDER is on its size with center point at z=0
uint8 CYLINDER_HEIGHT=0
uint8 CYLINDER_RADIUS=1

uint8 CONE_HEIGHT=0
uint8 CONE_RADIUS=1
