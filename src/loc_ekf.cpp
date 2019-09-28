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

#include "std_msgs/Int8.h"

using namespace std;
using namespace Eigen;
using namespace PointMatcherSupport;

#define ODOM_TIME 0.02

class loc_ekf
{

    typedef PointMatcher<float> PM;
    typedef PM::DataPoints DP;

public:
    loc_ekf(ros::NodeHandle &n);
    ~loc_ekf();
    ros::NodeHandle& n;

    ros::Subscriber mag_pose_sub;
    ros::Subscriber icp_pose_sub;
    ros::Subscriber odom_sub;
    ros::Subscriber receive_laser_sub;

    tf::TransformListener tf_listener_base2world;
    tf::TransformBroadcaster tf_broader_base2world;

    // three messages
    void gotMag(const geometry_msgs::PointStamped& magMsgIn);
    void gotOdom(const nav_msgs::Odometry& odomMsgIn);

    void gotLaser1(const sensor_msgs::PointCloud2& cloudMsgIn);
    void gotLaser2(const sensor_msgs::PointCloud2& cloudMsgIn);
    void gotIcp(const geometry_msgs::Pose2D& icpMsgIn);
    void gotLaserIn(const std_msgs::Int8& receiveLaserCnt);

    int magCnt;
    int mag_init_int;

    // EKF sth.
    VectorXf veh_sta;

    MatrixXf conv;
    MatrixXf jacob_F;
    MatrixXf jacob_H;
    MatrixXf noise_R; // motion noise
    MatrixXf noise_P; // laser noise
    MatrixXf gain_K;
    MatrixXf matrix_I;
    MatrixXf matrix_A;

    void finalPublish(VectorXf pose);

    PM::TransformationParameters Pose2DToPM3D(Vector3f pose);
    Matrix3f Pose2DToRT3D(Vector3f pose);
    Vector3f RT3DToPose2D(Matrix3f RT);

    bool init_flag;

    int odomCnt;

    int lastOdomSeq;
    bool odomInitFlag;
    int conLaserCnt;
};

loc_ekf::~loc_ekf()
{}

loc_ekf::loc_ekf(ros::NodeHandle& n):
    n(n),
    mag_init_int(getParam<int>("mag_init_int", 10))
{
    conLaserCnt = 1; // from the loc_reg start

    magCnt = 0; // count in initial
    veh_sta.conservativeResize(6);
    veh_sta << 0,0,0,0,0,0;

    noise_R = 1*MatrixXf::Identity(6,6); noise_R.bottomLeftCorner(3,3).setZero();
    noise_P = 0.1*MatrixXf::Identity(3,3); noise_P(2,2) = 0.01;
    matrix_I = 1*MatrixXf::Identity(6,6);
    gain_K = MatrixXf::Zero(6,3);
    matrix_A = 0*MatrixXf::Identity(6,3);
    matrix_A.topLeftCorner(3,3).setIdentity(); matrix_A.bottomRightCorner(3,3).setIdentity();

    // jacob = identity for icp pose
    jacob_H = MatrixXf::Zero(3,6); //jacob_H.topLeftCorner(3,3).setZero();

    init_flag = false;

    odomCnt = 0;
    odomInitFlag = false;

    mag_pose_sub = n.subscribe("mag_pose", 1, &loc_ekf::gotMag, this);
    icp_pose_sub = n.subscribe("icp_pose", 1, &loc_ekf::gotIcp, this);
    odom_sub = n.subscribe("wheel_odom", 1, &loc_ekf::gotOdom, this);
    receive_laser_sub = n.subscribe("receive_laser", 1, &loc_ekf::gotLaserIn, this);
}

void loc_ekf::gotMag(const geometry_msgs::PointStamped &magMsgIn)
{
    cout<<"-------------------Mag-------------------"<<endl;
    cout<<"Time:    "<<magMsgIn.header.stamp<<endl;
    cout<<"GT:  "<<magMsgIn.point.x<<"  "<<magMsgIn.point.y<<"  "<<magMsgIn.point.z<<endl;

    if(!init_flag)
    {
        veh_sta(0) = magMsgIn.point.x;   veh_sta(1) = magMsgIn.point.y; veh_sta(2) = magMsgIn.point.z;
        veh_sta(3) = magMsgIn.point.x;   veh_sta(4) = magMsgIn.point.y; veh_sta(5) = magMsgIn.point.z;

        conv = 0*MatrixXf::Identity(6,6);

        init_flag = true;
        return;
    }

    if(magCnt <= mag_init_int )
    {
        cout<<"USE"<<endl;

//        Vector3f mag_pose( magMsgIn.point.x, magMsgIn.point.y, magMsgIn.point.z);

//        Matrix3f K_ = jacob_H * conv * jacob_H.transpose() + noise_Q;
//        gain_K = conv * jacob_H.transpose() * K_.inverse();
//        veh_sta = veh_sta + gain_K * (mag_pose - jacob_H * veh_sta);
//        conv = (matrix_I - gain_K*jacob_H) * conv;

        /// ?XX?
        veh_sta(0) = magMsgIn.point.x;   veh_sta(1) = magMsgIn.point.y; veh_sta(2) = magMsgIn.point.z;
        veh_sta(3) = magMsgIn.point.x;   veh_sta(4) = magMsgIn.point.y; veh_sta(5) = magMsgIn.point.z;

        conv = 0*MatrixXf::Identity(6,6);

        this->finalPublish(veh_sta);

    }

    magCnt ++;
}

void loc_ekf::gotOdom(const nav_msgs::Odometry &odomMsgIn)
{
    cout<<"-------------------odom-------------------"<<endl;
    cout<<"Time:    "<<odomMsgIn.header.stamp<<endl;

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

    // motion
    veh_sta(0) = cos(veh_sta(2))*deltaTime*odomMsgIn.twist.twist.linear.x - sin(veh_sta(2))*deltaTime*odomMsgIn.twist.twist.linear.y + veh_sta(0);
    veh_sta(1) = sin(veh_sta(2))*deltaTime*odomMsgIn.twist.twist.linear.x + cos(veh_sta(2))*deltaTime*odomMsgIn.twist.twist.linear.y + veh_sta(1);
    veh_sta(2) = veh_sta(2) + odomMsgIn.twist.twist.angular.x*deltaTime;

    // nothing on the spesific past state
    veh_sta(3) = veh_sta(3) + 0;
    veh_sta(4) = veh_sta(4) + 0;
    veh_sta(5) = veh_sta(5) + 0;

    jacob_F = 1*MatrixXf::Identity(6,6);
    jacob_F(0,2) = -sin(lastOrient)*deltaTime*odomMsgIn.twist.twist.linear.x - cos(lastOrient)*deltaTime*odomMsgIn.twist.twist.linear.y;
    jacob_F(1,2) = cos(lastOrient)*deltaTime*odomMsgIn.twist.twist.linear.x - sin(lastOrient)*deltaTime*odomMsgIn.twist.twist.linear.y;

    conv = jacob_F*conv*jacob_F.transpose() + this->noise_R;

    this->finalPublish(veh_sta);

    lastOdomSeq = currentOdomSeq;
}

void loc_ekf::gotLaserIn(const std_msgs::Int8 &receiveLaserCnt)
{
    if(receiveLaserCnt.data == conLaserCnt)
    {
        cout<<"-------------------Laser12-------------------"<<endl;
        cout<<"Time:    "<<ros::Time::now()<<endl;
        cout<<"Con-Cnt: "<<conLaserCnt<<endl;

        conLaserCnt ++;

        // set the state augmentation of EKF

        veh_sta = matrix_A * veh_sta.head(3);
        conv = matrix_A * conv.topLeftCorner(3,3) * matrix_A.transpose();

        cout<<conv<<endl;

    }
}

void loc_ekf::gotIcp(const geometry_msgs::Pose2D &icpMsgIn)
{
    cout<<"-------------------ICP-------------------"<<endl;
    cout<<"Time:    "<<ros::Time::now()<<endl;

    // laser measure
    Vector3f lastPoseV(icpMsgIn.x, icpMsgIn.y, icpMsgIn.theta);
    Matrix3f lastPoseM = this->Pose2DToRT3D(lastPoseV);

    Matrix3f currPoseM = this->Pose2DToRT3D(veh_sta.head(3));

    Matrix3f measureM = lastPoseM.inverse() * currPoseM;
    Vector3f measureV = this->RT3DToPose2D(measureM);


    // self states
    Matrix3f lastPoseStaM = this->Pose2DToRT3D(veh_sta.tail(3));

    Matrix3f zM = lastPoseStaM.inverse() * currPoseM;
    Vector3f zV = this->RT3DToPose2D(zM);

    Matrix2f temp1;
    temp1 << -lastPoseStaM(1,0), lastPoseStaM(0,0),
            -lastPoseStaM(0,0), lastPoseStaM(1,0);

    Vector2f temp2(currPoseM(0,2)-lastPoseStaM(0,2), currPoseM(1,2)-lastPoseStaM(1,2));
    Vector2f temp3 = temp1 * temp2;

    Matrix3f H1 = 0*MatrixXf::Zero(3,3);  H1(2,2) = 1;
    H1.topLeftCorner(2,2) = lastPoseStaM.block(0,0,2,2).transpose();

    Matrix3f H2 = 0*MatrixXf::Zero(3,3);  H2(2,2) = -1;
    H2.topLeftCorner(2,2) = -1*lastPoseStaM.block(0,0,2,2).transpose();
    H2.topRightCorner(2,1) = temp3;

    jacob_H << H1, H2;

    Matrix3f S = jacob_H * conv * jacob_H.transpose() + noise_P;
    gain_K = conv * jacob_H.transpose() * S.inverse();

    // ++
    veh_sta = veh_sta + gain_K * (measureV - zV);

    conv = (matrix_I - gain_K*jacob_H) * conv;


    cout<<"Conv:    "<<endl;
    cout<<conv<<endl;
    cout<<"Gain K:  " <<endl;
    cout<<gain_K<<endl;

    this->finalPublish(veh_sta);
}

void loc_ekf::finalPublish(VectorXf pose)
{
    cout<<"Current Time:    "<<ros::Time::now()<<endl;
    cout<<"VEH_STA:  "<<pose.transpose()<<endl;
    PM::TransformationParameters T_base2world = this->Pose2DToPM3D(pose.head(3));
    tf_broader_base2world.sendTransform(PointMatcher_ros::eigenMatrixToStampedTransform<float>(T_base2world, "world", "base_footprint", ros::Time::now()));
}

loc_ekf::PM::TransformationParameters loc_ekf::Pose2DToPM3D(Vector3f pose)
{
    PM::TransformationParameters t = PM::TransformationParameters::Identity(4, 4);
    t(0,3) = pose(0); t(1,3) = pose(1);
    AngleAxisf V1(pose(2), Vector3f(0, 0, 1));
    Matrix3f R1 = V1.toRotationMatrix();
    t.block(0,0,3,3) = R1;
    return t;
}

Matrix3f loc_ekf::Pose2DToRT3D(Vector3f pose)
{
    Matrix3f RT;
    RT << cos(pose(2)), -sin(pose(2)), pose(0),
          sin(pose(2)), cos(pose(2)),pose(1),
          0, 0, 1;
    return RT;
}

Vector3f loc_ekf::RT3DToPose2D(Matrix3f RT)
{
    Vector3f pose(RT(0,2), RT(1,2), std::atan2(RT(1,0), RT(0,0)));
    return pose;
}

int main(int argc, char **argv)
{

    // INIT
    ros::init(argc, argv, "loc_ekf");
    ros::NodeHandle n;

    loc_ekf loc_ekf_(n);

    ros::MultiThreadedSpinner spinner(1);
    spinner.spin();

}
