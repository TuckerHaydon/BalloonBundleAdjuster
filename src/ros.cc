/* Author: Tucker Haydon */

#include <ros/ros.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "balloon_estimator");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }


    return EXIT_SUCCESS;
}
