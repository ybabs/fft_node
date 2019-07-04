#Timestamped data with GPS of the Drone

Header header
float64 latitude  #degree
float64 longitude #degree
float64 altitude  #WGS 84 reference ellipsoid fused by barometer
uint32 fc_timestamp_us
uint32 fc_utc_yymmdd
uint32 fc_utc_hhmmss
std_msgs/Float32MultiArray fftAmplitude
