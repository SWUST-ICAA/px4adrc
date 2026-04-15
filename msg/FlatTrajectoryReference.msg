std_msgs/Header header
# PX4 world frame NED.
geometry_msgs/Point position_ned
# PX4 world frame NED.
geometry_msgs/Vector3 velocity_ned
# PX4 world frame NED.
geometry_msgs/Vector3 acceleration_ned
# Desired body-frame angular rate in PX4 FRD.
geometry_msgs/Vector3 body_rates_frd
# Desired body-frame torque feedforward in PX4 FRD.
geometry_msgs/Vector3 body_torque_frd
# Desired heading in PX4 world frame NED.
float32 yaw
