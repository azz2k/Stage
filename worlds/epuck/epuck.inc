define epuckir sensor
(
  size [0.002 0.007 0.004]
  range [0 0.115]
  fov 40
  samples 3
  color_rgba [0 1 0 0.2]
)

define epuck_ir ranger
(
  epuckir(pose [0.030  -0.010  0 342.8])
  epuckir(pose  [0.022  -0.025  0 314.2])	
  epuckir(pose  [ 0.0    -0.031  0  270])
  epuckir(pose  [ -0.03   -0.015  0 208.5])
  epuckir(pose  [ -0.03  0.015   0 151.5])
  epuckir(pose  [ 0.0   0.031   0 90])
  epuckir(pose  [0.022 0.025   0 405.8])
  epuckir(pose  [0.03  0.01    0 377.2])
)

define epuck_blobfinder blobfinder
( 
  # blobfinder properties
  colors_count 1
  colors [ "red" ]
  image [ 60 60 ]
  range 2.0
  
  # model properties
  size [ 0.01 0.01 0.01 ]
)

define mainbody block(
  points 10
  point[0] [0.0350 0.0000]
  point[1] [0.0329 0.0120]
  point[2] [0.0268 0.0225]
  point[3] [-0.0268 0.0225]
  point[4] [-0.0329 0.0120]
  point[5] [-0.0350 0.0000]
  point[6] [-0.0329 -0.0120]
  point[7] [-0.0268 -0.0225]
  point[8] [0.0268 -0.0225]
  point[9] [0.0329 -0.0120]
  z [0 0.04]
  color "gray"
)

# top pcb
define mainpcb block(
  points 18
  point[0] [-0.0000 0.0350]
  point[1] [-0.0120 0.0329]
  point[2] [-0.0225 0.0268]
  point[3] [-0.0303 0.0175]
  point[4] [-0.0345 0.0061]
  point[5] [-0.0345 -0.0061]
  point[6] [-0.0303 -0.0175]
  point[7] [-0.0225 -0.0268]
  point[8] [-0.0120 -0.0329]
  point[9] [0.0000 -0.0350]
  point[10] [0.0120 -0.0329]
  point[11] [0.0225 -0.0268]
  point[12] [0.0303 -0.0175]
  point[13] [0.0345 -0.0061]
  point[14] [0.0345 0.0061]
  point[15] [0.0303 0.0175]
  point[16] [0.0225 0.0268]
  point[17] [0.0120 0.0329]
  z [0.04 0.0416]
  color "DarkGreen"
)

define turrent1 block(
  points 11
  point[0] [-0.035 0]
  point[1] [-0.028 -0.018 ]
  point[2] [0.011 -0.012 ]
  point[3] [0.026 -0.021 ]
  point[4] [0.027 -0.019 ]
  point[5] [0.015 -0.008 ]
  point[6] [0.015 0.008 ]
  point[7] [0.027 0.019 ]
  point[8] [0.026 0.021 ]
  point[9] [0.011 0.012 ]
  point[10] [-0.028 0.018 ]
  z [0.048 0.0496]
  color "LightGreen"
)
define dummybody block(
  points 4
  point[0] [0.04 0.04]
  point[1] [-0.04 0.04]
  point[2] [-0.04 -0.04]
  point[3] [0.04 -0.04]
  z [0.0 0.0496]
  color "green"
)

define  epuck_base position
(
  
  origin [0 0 0 0]
  mass 0.3
  
  color "red"
  drive "diff"
  gui_nose 1
  obstacle_return 1
  laser_return 1
  ranger_return 1
  blob_return 1
  fiducial_return 1

  localization "gps"
  localization_origin [0 0 0 0]
  
  #some blocks  	
  #mainbody()
  #mainpcb()
  
)

define epuck epuck_base
(
  size [0.07 0.07 0.05]
  mainbody()
  mainpcb()
  turrent1()
  epuck_ir(pose [ 0 0 -0.025 0 ])
  #camera( size [0.003 0.006 0.006] pose [ 0.034  0 -0.02 0 ]  range [ 0.05 1.0 ] resolution [ 100 100 ] fov [ 60 40 ] pantilt [ 0 0 ] )
  #epuck_blobfinder(pose [ 0 0 -0.01 0 ])
#  ctrl "epuckAvoidance"  
)

