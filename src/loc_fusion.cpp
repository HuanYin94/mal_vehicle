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

class loc_fusion
{

    typedef PointMatcher<float> PM;
    typedef PM::DataPoints DP;

public:
    loc_fusion(ros::NodeHandle &n);
    ~loc_fusion();
    ros::NodeHandle& n;

    ros::Subscriber mag_pose_sub;
    ros::Subscriber odom_sub;

    tf::TransformListener tf_listener_base2world;
    tf::TransformBroadcaster tf_broader_base2world;

    // three messages
    void gotMag(const geometry_msgs::PointStamped& magMsgIn);
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

    void finalPublish(Vector3f pose, const ros::Time& stamp);

    PM::TransformationParameters Pose2DToRT3D(Vector3f input);

    bool init_flag;

    bool process_flag;
    int odomCnt;

    int lastOdomSeq;
    bool odomInitFlag;

    /// from laser_reg.cpp

    ros::Subscriber laser1_sub;
    ros::Subscriber laser2_sub;

    void gotLaser1(const sensor_msgs::PointCloud2& cloudMsgIn);
    void gotLaser2(const sensor_msgs::PointCloud2& cloudMsgIn);

    DP mapCloud;
    string loadMapName;

    DP laserCloud1, laserCloud2;
    unique_ptr<PM::Transformation> transformation;

    tf::TransformListener tf_listener_calib_lasers;
    tf::TransformListener tf_listener_calib_laser2basefootprint;

    ros::Publisher mapPublisher;

    string inputFilterYamlName;
    PM::DataPointsFilters inputFilters;

    void registration(DP cloudIn, const ros::Time& stamp);

    PM::ICPSequence icp;
    string icpYamlName;

    PM::TransformationParameters T_laser12;
    PM::TransformationParameters T_laser22base;
    PM::TransformationParameters T_base2world;
    PM::TransformationParameters T_laser22world;

    long int mapPubCnt;

    Vector3f PMTransform2Pose2D(PM::TransformationParameters RT);

    float icp_xy_max, icp_yaw_max, icp_overlap_min;

    int laserCnt_1, laser_cyc;

    bool processFlag;

    ros::Publisher poseMsgPublisher;
};

loc_fusion::~loc_fusion()
{}

loc_fusion::loc_fusion(ros::NodeHandle& n):
    n(n),
    mag_init_int(getParam<int>("mag_init_int", 0)),
    loadMapName(getParam<string>("loadMapName", ".")),
    transformation(PM::get().REG(Transformation).create("RigidTransformation")),
    inputFilterYamlName(getParam<string>("inputFilterYamlName", ".")),
    icpYamlName(getParam<string>("icpYamlName", ".")),
    icp_xy_max(getParam<float>("icp_xy_max", 0)),
    icp_yaw_max(getParam<float>("icp_yaw_max", 0)),
    icp_overlap_min(getParam<float>("icp_overlap_min", 0)),
    laser_cyc(getParam<int>("laser_cyc", 0))
{
    /// ICP_reg init
    laserCnt_1=0;
    processFlag = false;

    /// prepare, load yamls
    // set icp
    ifstream ifss(icpYamlName.c_str());
    if (ifss.good())
    {
        icp.loadFromYaml(ifss);
        cout<<"SET"<<endl;
    }
    else
    {
        cout<<"UNSET"<<endl;
    }

    mapCloud = DP::load(loadMapName);
    if(!icp.hasMap())
    {
        icp.setMap(mapCloud);
    }
    mapPublisher = n.advertise<sensor_msgs::PointCloud2>( "map", 1 );
    mapPubCnt = 0;

    // set filter
    ifstream ifs(inputFilterYamlName.c_str());
    if (ifs.good())
    {
        inputFilters = PM::DataPointsFilters(ifs);
    }

    // pose msg publisher
    poseMsgPublisher = n.advertise<geometry_msgs::PointStamped>( "ekf_pose", 1);


    magCnt = 0; // count in initial
    veh_sta << 0,0,0;

    noise_R = 0.1*Matrix3f::Identity();  noise_R(3,3) = 1.0; // motion noise
    noise_P = 0.1*Matrix3f::Identity();// laser noise
    noise_Q = 0.01*Matrix3f::Identity();  // mag noise
    matrix_I = 1*Matrix3f::Identity();

    // jacob = identity for icp & mag
    jacob_H = 1*Matrix3f::Identity();

    init_flag = false;

    process_flag = false;
    odomCnt = 0;
    odomInitFlag = false;

    mag_pose_sub = n.subscribe("mag_pose", 1, &loc_fusion::gotMag, this);
    laser1_sub = n.subscribe("/velodyne1/velodyne_points", 1, &loc_fusion::gotLaser1, this);
    laser2_sub = n.subscribe("/velodyne2/velodyne_points", 1, &loc_fusion::gotLaser2, this);
    odom_sub = n.subscribe("wheel_odom", 1, &loc_fusion::gotOdom, this);

}

void loc_fusion::gotMag(const geometry_msgs::PointStamped &magMsgIn)
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

//    cout<<"-------------------Mag-------------------"<<endl;
//    cout<<"GT:  "<<magMsgIn.point.x<<"  "<<magMsgIn.point.y<<"  "<<magMsgIn.point.z<<endl;

    if(magCnt <= mag_init_int )
    {
        cout<<">>>>IN MAG"<<endl;

        Vector3f mag_pose( magMsgIn.point.x, magMsgIn.point.y, magMsgIn.point.z);

        Matrix3f K_ = jacob_H * conv * jacob_H.transpose() + noise_Q;
        gain_K = conv * jacob_H.transpose() * K_.inverse();
        veh_sta = veh_sta + gain_K * (mag_pose - jacob_H * veh_sta);
        conv = (matrix_I - gain_K*jacob_H) * conv;

        this->finalPublish(veh_sta, magMsgIn.header.stamp);

    }

    magCnt ++;
}

void loc_fusion::gotOdom(const nav_msgs::Odometry &odomMsgIn)
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

//    cout<<"AFTER:     "<<veh_sta.transpose()<<endl;N T

//    cout<<"Conv:    "<<conv<<endl;

    this->finalPublish(veh_sta, odomMsgIn.header.stamp);

    lastOdomSeq = currentOdomSeq;
}

void loc_fusion::gotLaser1(const sensor_msgs::PointCloud2& cloudMsgIn)
{
    if((laserCnt_1%laser_cyc)!=0)
    {
        laserCnt_1++; processFlag = false;
        return;
    }
    laserCnt_1++; processFlag = true;
    cout<<"L_CNT_1: "<<laserCnt_1<<endl;

    cout<<"-------------------laser1-------------------"<<endl;

    this->laserCloud1 = DP(PointMatcher_ros::rosMsgToPointMatcherCloud<float>(cloudMsgIn));


    this->T_laser12 = PointMatcher_ros::eigenMatrixToDim<float>(
               PointMatcher_ros::transformListenerToEigenMatrix<float>(
               this->tf_listener_calib_lasers,
               "laser1",
               "laser2",
               ros::Time::now()
           ), laserCloud1.getHomogeneousDim());

    laserCloud1 = transformation->compute(laserCloud1, this->T_laser12.inverse());
}

void loc_fusion::gotLaser2(const sensor_msgs::PointCloud2& cloudMsgIn)
{
    if(laserCloud1.features.cols() == 0 || !processFlag)
    {
        return;
    }

    cout<<"-------------------laser2-------------------"<<endl;

    double t0 = ros::Time::now().toSec();

    this->laserCloud2 = DP(PointMatcher_ros::rosMsgToPointMatcherCloud<float>(cloudMsgIn));

    DP conCloud = laserCloud2;

    // print out the time
//    cout<<laserCloud1<<endl;

    // concatenate
    conCloud.concatenate(laserCloud1);

    // filter the input cloud
    inputFilters.apply(conCloud);

    ///do icp
    this->registration(conCloud, cloudMsgIn.header.stamp);

    double t1 = ros::Time::now().toSec();
    cout<<"ICP total time cost:   "<<t1-t0<<" seconds."<<endl;
}

void loc_fusion::registration(DP cloudIn, const ros::Time& stamp)
{
    cout<<"-------------------registration-------------------"<<endl;

    // publish map
    if(mapPubCnt%10 == 0)
        mapPublisher.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>(mapCloud, "world", ros::Time::now()));
    mapPubCnt++;

    /// if do icp registration ?
//    return;
    /// yh 0_0

//    ROS_INFO_STREAM("input points' num:  " << cloudIn.features.cols());

    this->T_laser22base = PointMatcher_ros::eigenMatrixToDim<float>(
               PointMatcher_ros::transformListenerToEigenMatrix<float>(
               this->tf_listener_calib_laser2basefootprint,
               "base_footprint",
               "laser2",
                ros::Time(0)
           ), cloudIn.features.rows());

    cout<<ros::Time::now()<<endl;

    this->T_base2world = PointMatcher_ros::eigenMatrixToDim<float>(
               PointMatcher_ros::transformListenerToEigenMatrix<float>(
               this->tf_listener_base2world,
               "world",
               "base_footprint",
               ros::Time(0)
           ), cloudIn.features.rows());

    T_laser22world = T_base2world * T_laser22base;

//    cout<<T_laser22base<<endl;
//    cout<<" "<<endl;
//    cout<<T_base2world<<endl;
//    cout<<" "<<endl;
//    cout<<T_laser22world<<endl;

    try
    {
//        double t0 = ros::Time::now().toSec();
        PM::TransformationParameters T_laser22world_new = icp(cloudIn, T_laser22world);
//        double t1 = ros::Time::now().toSec();
//        cout<<"Registration time cost:   "<<t1-t0<<" seconds."<<endl;

//        cout<<T_laser22world<<endl;
//        cout<<""<<endl;
//        cout<<T_laser22world_new<<endl;

        ROS_INFO_STREAM("icp over_lap:  " << icp.errorMinimizer->getOverlap());

        PM::TransformationParameters T_base2world_new = T_laser22world_new * T_laser22base.inverse();

        /// judge the icp convergence-error
        PM::TransformationParameters T_base2world_relative = T_base2world.inverse() * T_base2world_new;
        Vector3f pose_relative = this->PMTransform2Pose2D(T_base2world_relative);

        cout<<"ICP-Relative:  "<< pose_relative.transpose()<<endl;

        // magic numbrer
        if(icp.errorMinimizer->getOverlap()<icp_overlap_min || abs(pose_relative(0))>icp_xy_max || abs(pose_relative(1))>icp_xy_max || abs(pose_relative(2))>icp_yaw_max)
        {
            cout<<"NO-SENT"<<endl;
            return;
        }

        ///no tf publish, send as an observation message to ekf-loc
        Vector3f icp_pose = this->PMTransform2Pose2D(T_base2world_new);

        ///EKF part, can be //

        Matrix3f K_ = jacob_H * conv * jacob_H.transpose() + noise_P;
        gain_K = conv * jacob_H.transpose() * K_.inverse();
        veh_sta = veh_sta + gain_K * (icp_pose - jacob_H * veh_sta);
        conv = (matrix_I - gain_K*jacob_H) * conv;

        cout<<"veh:     "<<veh_sta.transpose()<<endl;
        cout<<"Conv:    "<<conv<<endl;
        cout<<"gainK:    "<<gain_K<<endl;

        this->finalPublish(veh_sta, stamp);

        /// up

    }
    catch (PM::ConvergenceError error)
    {
        ROS_ERROR_STREAM("[ICP] failed to converge: " << error.what());
        return;
    }

}

Vector3f loc_fusion::PMTransform2Pose2D(PM::TransformationParameters RT)
{
    Vector3f p;
    p(0) = RT(0,3); p(1) = RT(1,3); p(2) = std::atan2(RT(1,0), RT(0,0));
    return p;
}


void loc_fusion::finalPublish(Vector3f pose, const ros::Time& stamp)
{
    // print out
    cout<<"Current Time:    "<<ros::Time::now()<<endl;
    cout<<"PUBLISH IN TF-TREE:  "<<pose.transpose()<<endl;

    // publish in tf
    PM::TransformationParameters T_base2world = this->Pose2DToRT3D(pose);
    tf_broader_base2world.sendTransform(PointMatcher_ros::eigenMatrixToStampedTransform<float>(T_base2world, "world", "base_footprint", ros::Time::now()));

    // publish a certain ros message, for recording
    geometry_msgs::PointStamped ekf_pose;
    ekf_pose.header.stamp = stamp;
    ekf_pose.point.x = pose(0);
    ekf_pose.point.y = pose(1);
    ekf_pose.point.z = pose(2);
    poseMsgPublisher.publish(ekf_pose);
}

loc_fusion::PM::TransformationParameters loc_fusion::Pose2DToRT3D(Vector3f input)
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
    ros::init(argc, argv, "loc_fusion");
    ros::NodeHandle n;

    loc_fusion loc_fusion_(n);

    ros::MultiThreadedSpinner spinner(1);
    spinner.spin();

}
