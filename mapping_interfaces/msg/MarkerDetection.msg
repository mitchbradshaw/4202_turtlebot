#aruco marker stats message file for node communication
 
int32 id                      # ArUco marker ID
geometry_msgs/Point position  # marker centre, metres
float64 range_m               # distance camera -> marker centre, metres
float64 pixel_area            # marker outline area in the image, px^2
