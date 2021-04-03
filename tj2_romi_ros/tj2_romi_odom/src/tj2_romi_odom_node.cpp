#include "tj2_romi_odom/tj2_romi_odom.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tj2_romi_odom");
    ros::NodeHandle nh;

    TJ2RomiOdom broadcaster(&nh);
    int err = broadcaster.run();

    return err;
}