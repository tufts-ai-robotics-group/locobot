import math
import time
from interbotix_xs_modules.locobot import InterbotixLocobotXS
from interbotix_perception_modules.apriltag import InterbotixAprilTagInterface

# This script uses the perception pipeline to pick up objects and place them in some virtual basket on the left side of the robot
# It also uses the AR tag on the arm to get a better idea of where the arm is relative to the camera (though the URDF is pretty accurate already).
#
# To get started, open a terminal and type...
# 'roslaunch interbotix_xslocobot_control xslocobot_python.launch robot_model:=locobot_wx200 use_perception:=true'
# Then change to this directory and type 'python pick_place_no_armtag.py'

def main():
    bot = InterbotixLocobotXS("locobot_wx200", arm_model="mobile_wx200")
    # move camera such that it's tilting down
    bot.camera.pan_tilt_move(0, 0.75)

    # move the arm to the sleep pose
    # bot.arm.set_ee_pose_components(x=0.3, z=0.2) # safe pose
    # bot.arm.go_to_sleep_pose()

    # Detection of the box with april tags
    ati = InterbotixAprilTagInterface(
                    apriltag_ns="locobot" + "/apriltag",
                    init_node=False,
                    verbose=True)
    print("April Tag Interface")
    ati.valid_tags.append(0)
    pose = ati.find_pose()
    print("april tag position:", pose.position)
    print()

    # x_b, y_b, z_b = pose.position
    # x_b, y_b, z_b = pose.position.x, pose.position.y, pose.position.z
    # transform = bot.armtag.find_ref_to_arm_base_transform(position_only=True)
    # print(transform)
    # transform = bot.armtag.get_transform(tfBuffer = , target_frame = , source_frame = )
    # bot.arm.set_ee_pose_components(x=x_b, y=y_b, z=z_b+0.1, pitch=0.5)
    # time.sleep(10)
    # bot.arm.set_ee_pose_components(x=0.3, z=0.2)
    # bot.arm.go_to_sleep_pose()
    # return

    # move the robot back so it's centered and open the gripper
    bot.arm.set_ee_pose_components(x=0.3, z=0.2, moving_time=1.5)
    bot.gripper.open()

    # get the positions of any clusters present w.r.t. the 'locobot/arm_base_link'
    # sort the clusters such that they appear from left-to-right w.r.t. the 'locobot/arm_base_link'
    success, clusters = bot.pcl.get_cluster_positions(ref_frame="locobot/arm_base_link", sort_axis="z", reverse=True)
    print("clusters:", clusters)

    while len(clusters) > 0:
        cluster = clusters[0]
        bot.arm.set_ee_pose_components(x=0.3, z=0.2)
        x, y, z = cluster["position"]
        bot.arm.set_ee_pose_components(x=x, y=y, z=z+0.05, pitch=0.8)
        time.sleep(0.2)
        bot.arm.set_ee_pose_components(x=x, y=y, z=z, pitch=0.8)
        time.sleep(0.2)
        bot.gripper.close()
        time.sleep(0.2)

        bot.arm.set_ee_pose_components(x=x, y=y, z=z+0.05, pitch=0.5)
        time.sleep(0.2)
        bot.arm.set_ee_pose_components(x=0.4, y=-0.3, z=0.2, moving_time=1.5)
        time.sleep(0.2)
        bot.gripper.open()
        time.sleep(0.2)
        print("did pickup", cluster)

        bot.arm.set_ee_pose_components(x=0.3, z=0.2, moving_time=1.5)
        bot.arm.go_to_sleep_pose()
        time.sleep(0.1)
        success, clusters = bot.pcl.get_cluster_positions(ref_frame="locobot/arm_base_link", sort_axis="y", reverse=True)

    bot.arm.set_ee_pose_components(x=0.3, z=0.2)
    bot.arm.go_to_sleep_pose()

if __name__=='__main__':
    main()
