#include "ros/ros.h"
#include "ros/console.h"

#include "eigen_conversions/eigen_msg.h"

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include "ros/publisher.h"
#include "ros/subscriber.h"

#include "pointmatcher_ros/transform.h"
#include "pointmatcher_ros/point_cloud.h"
#include "nabo/nabo.h"
#include "pointmatcher_ros/get_params_from_server.h"

#include <fstream>

using namespace std;
using namespace Eigen;
using namespace PointMatcherSupport;

class recorder
{

    typedef PointMatcher<float> PM;
    typedef PM::DataPoints DP;

public:
    recorder(ros::NodeHandle &n);
    ~recorder();
    ros::NodeHandle& n;


    string saveMagPoseName, saveOdomPoseName, saveLocPoseName;
    ros::Subscriber mag_pose_sub, odom_sub, ekf_pose_sub;
    tf::TransformListener tfListener;

    void gotMag(const geometry_msgs::PointStamped &magMsgIn);
    void gotOdom(const nav_msgs::Odometry &odomMsgIn);
    void gotEKFpose(const geometry_msgs::PointStamped &poseMsgIn);

    ofstream magPoseStream;
    ofstream odomPoseStream;
    ofstream locPoseStream;

    PM::TransformationParameters T_base2world;

    Vector3f RT3D2Pose2D(PM::TransformationParameters RT);

    PM::TransformationParameters Pose2DToRT3D(Vector3f input);
    float angleNorm(float head);

};

recorder::~recorder()
{}

recorder::recorder(ros::NodeHandle& n):
    n(n),
    saveMagPoseName(getParam<string>("saveMagPoseName", ".")),
    saveOdomPoseName(getParam<string>("saveOdomPoseName", ".")),
    saveLocPoseName(getParam<string>("saveLocPoseName", "."))
{
    // delete the existing files
    std::remove(saveMagPoseName.c_str());
    std::remove(saveOdomPoseName.c_str());
    std::remove(saveLocPoseName.c_str());

    mag_pose_sub = n.subscribe("mag_pose", 1, &recorder::gotMag, this);
    odom_sub = n.subscribe("wheel_odom", 1, &recorder::gotOdom, this);
    ekf_pose_sub = n.subscribe("ekf_pose", 1, &recorder::gotEKFpose, this);

}

void recorder::gotMag(const geometry_msgs::PointStamped &magMsgIn)
{

    /// mag pose
    magPoseStream.open(saveMagPoseName, ios::out|ios::ate|ios::app);
    magPoseStream << magMsgIn.header.stamp << "   "
                   << magMsgIn.point.x << "   "
                     << magMsgIn.point.y << "   "
                        << magMsgIn.point.z <<endl;
    magPoseStream.close();

//    Vector3f mag_measured;
    //    mag_measured << magMsgIn.point.x,
    //                    magMsgIn.point.y,
    //                    this->angleNorm(magMsgIn.point.z);

    //    PM::TransformationParameters T_mag = this->Pose2DToRT3D(mag_measured);

    //    locPoseStream.open(saveMagPoseName, ios::out|ios::ate|ios::app);
    //    locPoseStream << std::fixed << magMsgIn.header.stamp.toSec() << endl;
    //    locPoseStream << T_mag(0,0) << "    " << T_mag(0,1) << "    " << T_mag(0,2) << "    " << T_mag(0,3) << endl;
    //    locPoseStream << T_mag(1,0) << "    " << T_mag(1,1) << "    " << T_mag(1,2) << "    " << T_mag(1,3) << endl;
    //    locPoseStream << T_mag(2,0) << "    " << T_mag(2,1) << "    " << T_mag(2,2) << "    " << T_mag(2,3) << endl;
    //    locPoseStream << T_mag(3,0) << "    " << T_mag(3,1) << "    " << T_mag(3,2) << "    " << T_mag(3,3) << endl;
    //    locPoseStream.close();


}

void recorder::gotOdom(const nav_msgs::Odometry &odomMsgIn)
{
    Quaternionf Q(odomMsgIn.pose.pose.orientation.x, odomMsgIn.pose.pose.orientation.y, odomMsgIn.pose.pose.orientation.z, odomMsgIn.pose.pose.orientation.w);
    AngleAxisf V;
    V = Q;
    Vector3f As = V.axis();

    odomPoseStream.open(saveOdomPoseName, ios::out|ios::ate|ios::app);
    odomPoseStream << odomMsgIn.header.stamp << "   "
                    << odomMsgIn.pose.pose.position.x << "   "
                      << odomMsgIn.pose.pose.position.y << "   "
                         << As(2) <<endl;
    odomPoseStream.close();
}

void recorder::gotEKFpose(const geometry_msgs::PointStamped &poseMsgIn)
{
    /// estimated pose
    locPoseStream.open(saveLocPoseName, ios::out|ios::ate|ios::app);
    locPoseStream << poseMsgIn.header.stamp << "   "
                   << poseMsgIn.point.x << "   "
                     << poseMsgIn.point.y << "   "
                        << poseMsgIn.point.z <<endl;
    locPoseStream.close();
}


Vector3f recorder::RT3D2Pose2D(PM::TransformationParameters RT)
{
    Vector3f pose(RT(0,3), RT(1,3), std::atan2(RT(1,0), RT(0,0)));
    return pose;
}

recorder::PM::TransformationParameters recorder::Pose2DToRT3D(Vector3f input)
{
    PM::TransformationParameters t = PM::TransformationParameters::Identity(4, 4);
    t(0,3) = input(0); t(1,3) = input(1);
    AngleAxisf V1(input(2), Vector3f(0, 0, 1));
    Matrix3f R1 = V1.toRotationMatrix();
    t.block(0,0,3,3) = R1;
    return t;
}

float recorder::angleNorm(float head)
{
    // 0 ~ 360 ?
    while(head > 2*M_PI)
        head = head - 2*M_PI;
    while(head < 0)
        head = head + 2*M_PI;

    return head;
}

int main(int argc, char **argv)
{

    // INIT
    ros::init(argc, argv, "recorder");
    ros::NodeHandle n;

    recorder recorder_(n);

    ros::MultiThreadedSpinner spinner(1);
    spinner.spin();

}
