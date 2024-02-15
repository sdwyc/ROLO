Header header 

int32[] startRingIndex
int32[] endRingIndex

int32[]  pointColInd # point column index in range image
float32[] pointRange # point range

float32 startOrientation
float32 endOrientation
float32 orientationDiff

# Initial guess from front lidar odometry
float32 initialGuessX
float32 initialGuessY
float32 initialGuessZ
float32 initialGuessRoll
float32 initialGuessPitch
float32 initialGuessYaw
bool odomAvailable

# Point cloud messages
sensor_msgs/PointCloud2 cloud_projected  # original cloud
sensor_msgs/PointCloud2 extracted_corner    # extracted corner feature
sensor_msgs/PointCloud2 extracted_surface   # extracted surface feature
sensor_msgs/PointCloud2 extracted_normal   # extracted normal point(for back-end)
# sensor_msgs/PointCloud2 extracted_ground   # extracted ground

