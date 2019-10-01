#include "ros/ros.h"
#include "ros/console.h"

#include "eigen_conversions/eigen_msg.h"

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include "ros/publisher.h"
#include "ros/subscriber.h"

#include "geometry_msgs/Pose2D.h"

#include "pointmatcher_ros/transform.h"
#include "pointmatcher_ros/point_cloud.h"
#include "nabo/nabo.h"
#include "pointmatcher_ros/get_params_from_server.h"

#include <fstream>

#include "std_msgs/Int8.h"

using namespace std;
using namespace Eigen;
using namespace PointMatcherSupport;

class loc_reg
{

    typedef PointMatcher<float> PM;
    typedef PM::DataPoints DP;
    typedef PM::Matches Matches;

public:
    loc_reg(ros::NodeHandle &n);
    ~loc_reg();
    ros::NodeHandle& n;

    ros::Subscriber cloud_sub1, cloud_sub2;

    void gotCloud1(const sensor_msgs::PointCloud2& cloudMsgIn);
    void gotCloud2(const sensor_msgs::PointCloud2& cloudMsgIn);

    DP mapCloud;
    string loadMapName;

    DP laserCloud1, laserCloud2;
    unique_ptr<PM::Transformation> transformation;

    tf::TransformListener tf_listener_calib_lasers;
    tf::TransformListener tf_listener_calib_laser2basefootprint;
    tf::TransformListener tf_listener_base2world;
    tf::TransformBroadcaster tf_broader_base2world;

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

    geometry_msgs::Pose2D icp_pose_msg;
    ros::Publisher posePublisher;

    geometry_msgs::Pose2D  PMTransform2Pose2D(PM::TransformationParameters RT);

    float icp_xy_max, icp_yaw_max,icp_overlap_min;

    int laserCnt_1;

    int laser_cyc;

    ros::Publisher conLaserPublisher;

    bool processFlag;

    ros::Publisher receiveLaserPublisher;
    std_msgs::Int8 receivedLaser;

};

loc_reg::~loc_reg()
{}

loc_reg::loc_reg(ros::NodeHandle& n):
    n(n),
    loadMapName(getParam<string>("loadMapName", ".")),
    transformation(PM::get().REG(Transformation).create("RigidTransformation")),
    inputFilterYamlName(getParam<string>("inputFilterYamlName", ".")),
    icpYamlName(getParam<string>("icpYamlName", ".")),
    icp_xy_max(getParam<float>("icp_xy_max", 0)),
    icp_yaw_max(getParam<float>("icp_yaw_max", 0)),
    icp_overlap_min(getParam<float>("icp_overlap_min", 0)),
    laser_cyc(getParam<int>("laser_cyc", 0))
{
    laserCnt_1=0;
    processFlag = false;
    receivedLaser.data = 0;

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

    posePublisher = n.advertise<geometry_msgs::Pose2D>( "icp_pose", 1 );

    // publish the con-cloud
    conLaserPublisher = n.advertise<sensor_msgs::PointCloud2>( "con_laser", 1);

    receiveLaserPublisher = n.advertise<std_msgs::Int8>("receive_laser", 1);

    cloud_sub1 = n.subscribe("/velodyne1/velodyne_points", 1, &loc_reg::gotCloud1, this);
    cloud_sub2 = n.subscribe("/velodyne2/velodyne_points", 1, &loc_reg::gotCloud2, this);

}

void loc_reg::gotCloud1(const sensor_msgs::PointCloud2& cloudMsgIn)
{
    if((laserCnt_1%laser_cyc)!=0)
    {
        laserCnt_1++; processFlag = false;
        return;
    }
    cout<<"-------------------!!!!-------------------"<<endl;

    laserCnt_1++; processFlag = true;
    cout<<"L_CNT_1: "<<laserCnt_1<<endl;

    this->laserCloud1 = DP(PointMatcher_ros::rosMsgToPointMatcherCloud<float>(cloudMsgIn));

    double t_wait_0 = ros::Time::now().toSec();

    this->T_laser12 = PointMatcher_ros::eigenMatrixToDim<float>(
               PointMatcher_ros::transformListenerToEigenMatrix<float>(
               this->tf_listener_calib_lasers,
               "laser1",
               "laser2",
               ros::Time::now()
           ), laserCloud1.getHomogeneousDim());

    double t_wait_1 = ros::Time::now().toSec();
    cout<<"Wait_trans time cost:   "<<t_wait_1-t_wait_0<<" seconds."<<endl;


    double t_trans_0 = ros::Time::now().toSec();
    laserCloud1 = transformation->compute(laserCloud1, this->T_laser12.inverse());
    double t_trans_1 = ros::Time::now().toSec();
    cout<<"Laser1_trans time cost:   "<<t_trans_1-t_trans_0<<" seconds."<<endl;

}

void loc_reg::gotCloud2(const sensor_msgs::PointCloud2& cloudMsgIn)
{
    if(laserCloud1.features.cols() == 0 || !processFlag)
    {
        return;
    }

    // send the received signal to ekf node
    receivedLaser.data ++;
    receiveLaserPublisher.publish(receivedLaser);
    cout<<"Singal-Sent-Time:    "<<ros::Time::now()<<endl;

    // receive the transform tree
    this->T_laser22base = PointMatcher_ros::eigenMatrixToDim<float>(
               PointMatcher_ros::transformListenerToEigenMatrix<float>(
               this->tf_listener_calib_laser2basefootprint,
               "base_footprint",
               "laser2",
                ros::Time(0)
           ), laserCloud1.features.rows());

    this->T_base2world = PointMatcher_ros::eigenMatrixToDim<float>(
               PointMatcher_ros::transformListenerToEigenMatrix<float>(
               this->tf_listener_base2world,
               "world",
               "base_footprint",
               ros::Time::now()
           ), laserCloud1.features.rows());

    T_laser22world = T_base2world * T_laser22base;

    // process the laser_2
    this->laserCloud2 = DP(PointMatcher_ros::rosMsgToPointMatcherCloud<float>(cloudMsgIn));

    DP conCloud = laserCloud2;

    // concatenate
    double t_con_0 = ros::Time::now().toSec();
    conCloud.concatenate(laserCloud1);
    double t_con_1 = ros::Time::now().toSec();
    cout<<"Concatenate time cost:   "<<t_con_1-t_con_0<<" seconds."<<endl;

    // filter the input cloud
    double t_filter_0 = ros::Time::now().toSec();
    inputFilters.apply(conCloud);
    double t_filter_1 = ros::Time::now().toSec();
    cout<<"Filtering time cost:   "<<t_filter_1-t_filter_0<<" seconds."<<endl;

    // publish the con-cloud
    conLaserPublisher.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>(conCloud, "laser2", ros::Time::now()));

    ///do icp
    cout<<"ICP stamp:   "<<cloudMsgIn.header.stamp<<endl;
    this->registration(conCloud, cloudMsgIn.header.stamp);

}

void loc_reg::registration(DP cloudIn, const ros::Time& stamp)
{
    // publish map
    if(mapPubCnt%10 == 0)
        mapPublisher.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>(mapCloud, "world", ros::Time::now()));
    mapPubCnt++;

    /// if do icp registration ?
//    return;
    /// yh 0_0

    try
    {
        double t_reg_0 = ros::Time::now().toSec();
        PM::TransformationParameters T_laser22world_new = icp(cloudIn, T_laser22world);
        double t_reg_1 = ros::Time::now().toSec();
        cout<<"Registration time cost:   "<<t_reg_1-t_reg_0<<" seconds."<<endl;

//        cout<<T_laser22world<<endl;
//        cout<<""<<endl;
//        cout<<T_laser22world_new<<endl;

        ROS_INFO_STREAM("icp over_lap:  " << icp.errorMinimizer->getOverlap());

        PM::TransformationParameters T_base2world_new = T_laser22world_new * T_laser22base.inverse();

        /// judge the icp convergence-error
        PM::TransformationParameters T_base2world_relative = T_base2world.inverse() * T_base2world_new;
        geometry_msgs::Pose2D pose_relative = this->PMTransform2Pose2D(T_base2world_relative);

        cout<<"ICP-Relative:  "<<pose_relative.x<<"    "
           <<pose_relative.y<<"    "
          <<pose_relative.theta<<endl;

        // magic numbrer
        if(icp.errorMinimizer->getOverlap()<icp_overlap_min || abs(pose_relative.x)>icp_xy_max || abs(pose_relative.y)>icp_xy_max || abs(pose_relative.theta)>icp_yaw_max)
        {
            cout<<"NO-SENT"<<endl;
            return;
        }

        ///no tf publish, send as an observation message to ekf-loc
        icp_pose_msg = this->PMTransform2Pose2D(T_base2world_new);

        // print out the result, debug
        cout<<T_base2world(0,3)<<"    "<<T_base2world(1,3)<<endl;
        cout<<icp_pose_msg.x<<"    "<<icp_pose_msg.y<<"    "<<icp_pose_msg.theta<<endl;

        // publish this message
        posePublisher.publish(icp_pose_msg);
        cout<<"SENT"<<endl;

    }
    catch (PM::ConvergenceError error)
    {
        ROS_ERROR_STREAM("[ICP] failed to converge: " << error.what());
        return;
    }

}

geometry_msgs::Pose2D loc_reg::PMTransform2Pose2D(PM::TransformationParameters RT)
{
    geometry_msgs::Pose2D p;
    p.x = RT(0,3); p.y = RT(1,3); p.theta = std::atan2(RT(1,0), RT(0,0));
    return p;
}

int main(int argc, char **argv)
{

    // INIT
    ros::init(argc, argv, "loc_reg");
    ros::NodeHandle n;

    loc_reg loc_reg_(n);

    ros::MultiThreadedSpinner spinner(1);
    spinner.spin();

}
