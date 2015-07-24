# Desc: definition file for SyRoTek S1R robot
# Author: Jan Chudoba
# Date: 2009-01-21
# Last update: 2009-01-22

define s1r_front_ir ranger (
		alwayson 1
		scount 2
		spose[0] [ 0.078 0.0335 -0.1 0.03] # front left
		spose[1] [ 0.078 -0.0335 0 0.03] # front right
		sview [0.05 0.35 2] # [min max fov]
		ssize [0.04 0.01]
		)

define s1r_base_ir ranger (
		alwayson 1
		scount 3
		spose[0] [ -0.020 0.063 90 0.07] # left
		spose[1] [ -0.020 -0.063 -90 0.07] # right
		spose[2] [ -0.078 0 180 ] # rear
		sview [0.05 0.35 2] # [min max fov]
		ssize [0.04 0.01]
		)

define s1r_base_sonar ranger (
		alwayson 
		scount 3
		spose[0] [ -0.020 0.063 90 ] # left
		spose[1] [ -0.020 -0.063 -90 ] # right
		spose[2] [ -0.078 0 180 ] # rear
		sview [0.05 5.0 50] # [min max fov]
		ssize [0.04 0.01]
		)

define s1r_frontsensor_ir ranger (
		alwayson 1
		scount 3
		spose[0] [ 0.039 0.039 45 ] # left
		spose[1] [ 0.055 0 0 ] # center
		spose[2] [ 0.039 -0.039 -45 ] # rear
		sview [0.05 0.35 2] # [min max fov]
		ssize [0.04 0.01]
		)

define s1r_frontsensor_sonar ranger (
		alwayson 1
		scount 3
		spose[0] [ 0.039 0.039 45 ] # left
		spose[1] [ 0.055 0 0 ] # center
		spose[2] [ 0.039 -0.039 -45 ] # rear
		sview [0.05 5.0 50] # [min max fov]
		ssize [0.04 0.01]
		)

define S1R position 
(
#        localization "odom"
#   odom_error [0.0 0.01 0.0 0.01 0.0 0.01]

 color "blue"

 size [ 0.174 0.162 0.18]
 origin [0 0 0 0]
# gui_nose 1
 mass 2.0

# drive "diff"

 block(
	 points 8
	 point[7] [ 0.087  0.045 ]
	 point[6] [ 0.087 -0.045 ]
	 point[5] [ 0.050 -0.081 ]
	 point[4] [-0.050 -0.081 ]
	 point[3] [-0.087 -0.045 ]
	 point[2] [-0.087  0.045 ]
	 point[1] [-0.050  0.081 ]
	 point[0] [ 0.050  0.081 ]
	 z [0 0.06]
      )

 block(	
	 points 8
	 point[7] [ 0.075  0.030 ]
	 point[6] [ 0.075 -0.030 ]
	 point[5] [ 0.024 -0.059 ]
	 point[4] [-0.062 -0.059 ]
	 point[3] [-0.074 -0.050 ]
	 point[2] [-0.074  0.050 ]
	 point[1] [-0.062  0.059 ]
	 point[0] [ 0.024  0.059 ]
	 z [0.13 0.18]
     )

# block(	
#	 points 6
#	 point[5] [ 0.024 -0.059 ]
#	 point[4] [-0.062 -0.059 ]
#	 point[3] [-0.074 -0.050 ]
#	 point[2] [-0.074  0.050 ]
#	 point[1] [-0.062  0.059 ]
#	 point[0] [ 0.024  0.059 ]
#	 z [0.06 0.13]
#     )


	#s1r_front_ir()
	#s1r_base_ir()
	#s1r_base_sonar()
	#s1r_frontsensor_ir()
	#s1r_frontsensor_sonar()
	)

