
#define hokuyolaser ranger
#(
#  sensor(
  # laser-specific properties
  # factory settings for LMS200	
  #range_min 0.0
#  range [ 0 5 ]
#  fov 270.0
#  samples 682

  # generic model properties
#  color "blue"
#)
#  size [ 0.05 0.05 0.07 ]
#  alwayson 1
#)
define hokuyolaser ranger
(
  sensor(       
    # laser-specific properties
    # factory settings for LMS200  
    range [ 0.0  5.0 ]
    fov 270.0
    samples 682
  )
  model
  (
    # generic model properties
    size [ 0.07 0.07 0.05 ] # dimensions from LMS200 data sheet  
    color "blue"
  )
)
