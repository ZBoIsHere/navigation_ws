import rospy
import dynamic_reconfigure.client

rospy.init_node("myconfig_py", anonymous=True)
client = dynamic_reconfigure.client.Client("/move_base/TebLocalPlannerROS")

my_slow_params = {
    "max_vel_x": 0.1,
    "max_vel_x_backwards": 0.02,
    "max_vel_y": 0.08,
    "max_vel_theta": 0.2,
    "acc_lim_x": 0.08,
    "acc_lim_y": 0.03,
    "acc_lim_theta": 0.1,
    "yaw_goal_tolerance": 0.1,
    "xy_goal_tolerance": 0.1,
}

client.update_configuration(my_params)
