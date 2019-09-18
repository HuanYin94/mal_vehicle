#include "ros/ros.h"
#include "ros/console.h"

#include "eigen_conversions/eigen_msg.h"

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include "ros/publisher.h"
#include "ros/subscriber.h"

#include "geometry_msgs/Pose2D.h"

#include "pointmatcher_ros/transform.h"
#include "pointmatcher_ros/point_cloud.h"
#include "nabo/nabo.h"
#include "pointmatcher_ros/get_params_from_server.h"

#include <fstream>
#include <visualization_msgs/Marker.h>

using namespace std;
using namespace Eigen;
using namespace PointMatcherSupport;

#define ODOM_TIME 0.02

class loc_demo
{

    typedef PointMatcher<float> PM;
    typedef PM::DataPoints DP;

public:
    loc_demo(ros::NodeHandle &n);
    ~loc_demo();
    ros::NodeHandle& n;

    ros::Subscriber mag_pose_sub;
    ros::Subscriber icp_pose_sub;
    ros::Subscriber odom_sub;

    tf::TransformListener tf_listener_base2world;
    tf::TransformBroadcaster tf_broader_base2world;

    // three messages
    void gotMag(const geometry_msgs::PointStamped& magMsgIn);
    void gotIcp(const geometry_msgs::Pose2D& icpMsgIn);
    void gotOdom(const nav_msgs::Odometry& odomMsgIn);

    int magCnt;
    int mag_init_int;

    // EKF sth.
    Vector3f veh_sta;

    Matrix3f conv;
    Matrix3f jacob_F;
    Matrix3f jacob_H;
    Matrix3f noise_R; // motion noise
    Matrix3f noise_P; // laser noise
    Matrix3f noise_Q; // mag noise
    Matrix3f gain_K;
    Matrix3f matrix_I;

    void publishTF(Vector3f pose);

    PM::TransformationParameters Pose2DToRT3D(Vector3f input);

    bool init_flag;

    bool process_flag;
    int odomCnt;

    int lastOdomSeq;
    bool odomInitFlag;

};

loc_demo::~loc_demo()
{}

loc_demo::loc_demo(ros::NodeHandle& n):
    n(n),
    mag_init_int(getParam<int>("mag_init_int", 0))
{
    magCnt = 0; // count in initial
    veh_sta << 0,0,0;

    noise_R = 1*Matrix3f::Identity();
    noise_P = 0.1*Matrix3f::Identity();
    noise_Q = 0.001*Matrix3f::Identity();
    matrix_I = 1*Matrix3f::Identity();

    // jacob = identity for icp & mag
    jacob_H = 1*Matrix3f::Identity();

    init_flag = false;

    process_flag = false;
    odomCnt = 0;
    odomInitFlag = false;

    mag_pose_sub = n.subscribe("mag_pose", 1, &loc_demo::gotMag, this);
    icp_pose_sub = n.subscribe("icp_pose", 1, &loc_demo::gotIcp, this);
    odom_sub = n.subscribe("wheel_odom", 1, &loc_demo::gotOdom, this);

}

void loc_demo::gotMag(const geometry_msgs::PointStamped &magMsgIn)
{
    if(!init_flag)
    {
        veh_sta(0) = magMsgIn.point.x;
        veh_sta(1) = magMsgIn.point.y;
        veh_sta(2) = magMsgIn.point.z;

        conv = 0*Matrix3f::Identity();

        init_flag = true;
        return;
    }

    cout<<"-------------------Mag-------------------"<<endl;
    cout<<"GT:  "<<magMsgIn.point.x<<"  "<<magMsgIn.point.y<<"  "<<magMsgIn.point.z<<endl;

    if(magCnt <= mag_init_int )
    {
        cout<<">>>>IN MAG"<<endl;

        Vector3f mag_pose( magMsgIn.point.x, magMsgIn.point.y, magMsgIn.point.z);

        Matrix3f K_ = jacob_H * conv * jacob_H.transpose() + noise_Q;
        gain_K = conv * jacob_H.transpose() * K_.inverse();
        veh_sta = veh_sta + gain_K * (mag_pose - jacob_H * veh_sta);
        conv = (matrix_I - gain_K*jacob_H) * conv;

        this->publishTF(veh_sta);

    }

    magCnt ++;
}

void loc_demo::gotIcp(const geometry_msgs::Pose2D &icpMsgIn)
{
    cout<<"-------------------icp-------------------"<<endl;

    if(veh_sta(0) == 0)
        return;

    Vector3f icp_pose( icpMsgIn.x, icpMsgIn.y, icpMsgIn.theta);

    cout<<icp_pose.transpose()<<endl;

//    Matrix3f K_ = jacob_H * conv * jacob_H.transpose() + noise_P;
//    gain_K = conv * jacob_H.transpose() * K_.inverse();
//    veh_sta = veh_sta + gain_K * (icp_pose - jacob_H * veh_sta);
//    conv = (matrix_I - gain_K*jacob_H) * conv;

//    cout<<"veh:     "<<veh_sta.transpose()<<endl;
//    cout<<"Conv:    "<<conv<<endl;
//    cout<<"gainK:    "<<gain_K<<endl;

//    this->publishTF(veh_sta);
}

void loc_demo::gotOdom(const nav_msgs::Odometry &odomMsgIn)
{
    cout<<"-------------------odom-------------------"<<endl;

    if(veh_sta(0) == 0)
        return;

    /// get the time for calculation
    int currentOdomSeq;
    if(!odomInitFlag)
    {
        lastOdomSeq = odomMsgIn.header.seq;
        currentOdomSeq = lastOdomSeq + 1;
        odomInitFlag = true;
    }
    else
    {
        currentOdomSeq = odomMsgIn.header.seq;
    }
    double deltaTime = (currentOdomSeq - lastOdomSeq) * ODOM_TIME;

    float lastOrient = veh_sta(2);

//    cout<<"deltaTime:   "<<deltaTime<<endl;
//    cout<<"BEFORE:  "<<veh_sta.transpose()<<endl;

    veh_sta(0) = cos(veh_sta(2))*deltaTime*odomMsgIn.twist.twist.linear.x - sin(veh_sta(2))*deltaTime*odomMsgIn.twist.twist.linear.y + veh_sta(0);
    veh_sta(1) = sin(veh_sta(2))*deltaTime*odomMsgIn.twist.twist.linear.x + cos(veh_sta(2))*deltaTime*odomMsgIn.twist.twist.linear.y + veh_sta(1);
    veh_sta(2) = veh_sta(2) + odomMsgIn.twist.twist.angular.x*deltaTime;

    jacob_F << 1, 0, -sin(lastOrient)*deltaTime*odomMsgIn.twist.twist.linear.x - cos(lastOrient)*deltaTime*odomMsgIn.twist.twist.linear.y,
               0, 1, cos(lastOrient)*deltaTime*odomMsgIn.twist.twist.linear.x - sin(lastOrient)*deltaTime*odomMsgIn.twist.twist.linear.y,
               0, 0, 1;

    conv = jacob_F*conv*jacob_F.transpose() + this->noise_R;

//    cout<<"AFTER:     "<<veh_sta.transpose()<<endl;
//    cout<<"Conv:    "<<conv<<endl;

    this->publishTF(veh_sta);

    lastOdomSeq = currentOdomSeq;
}

void loc_demo::publishTF(Vector3f pose)
{
    cout<<"Current Time:    "<<ros::Time::now()<<endl;
    cout<<"PUBLISH IN TF-TREE:  "<<pose.transpose()<<endl;
    PM::TransformationParameters T_base2world = this->Pose2DToRT3D(pose);
    tf_broader_base2world.sendTransform(PointMatcher_ros::eigenMatrixToStampedTransform<float>(T_base2world, "world", "base_footprint", ros::Time::now()));
}

loc_demo::PM::TransformationParameters loc_demo::Pose2DToRT3D(Vector3f input)
{
    PM::TransformationParameters t = PM::TransformationParameters::Identity(4, 4);
    t(0,3) = input(0); t(1,3) = input(1);
    AngleAxisf V1(input(2), Vector3f(0, 0, 1));
    Matrix3f R1 = V1.toRotationMatrix();
    t.block(0,0,3,3) = R1;
    return t;
}


int main(int argc, char **argv)
{

    // INIT
    ros::init(argc, argv, "loc_demo");
    ros::NodeHandle n;

    loc_demo loc_demo_(n);

    ros::MultiThreadedSpinner spinner(1);
    spinner.spin();

}
