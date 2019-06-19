#include "ros/ros.h"
#include "ros/console.h"

#include "eigen_conversions/eigen_msg.h"

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

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

class laser_reg
{

    typedef PointMatcher<float> PM;
    typedef PM::DataPoints DP;
    typedef PM::Matches Matches;

public:
    laser_reg(ros::NodeHandle &n);
    ~laser_reg();
    ros::NodeHandle& n;

    ros::Subscriber cloud_sub1, cloud_sub2;

    void gotCloud1(const sensor_msgs::PointCloud2& cloudMsgIn);
    void gotCloud2(const sensor_msgs::PointCloud2& cloudMsgIn);

    DP mapCloud;
    string loadMapName;

    DP laserCloud1, laserCloud2, conCloud;
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

};

laser_reg::~laser_reg()
{}

laser_reg::laser_reg(ros::NodeHandle& n):
    n(n),
    loadMapName(getParam<string>("loadMapName", ".")),
    transformation(PM::get().REG(Transformation).create("RigidTransformation")),
    inputFilterYamlName(getParam<string>("inputFilterYamlName", ".")),
    icpYamlName(getParam<string>("icpYamlName", "."))
{
    /// prepare, load yamls
    mapCloud = DP::load(loadMapName);
    if(!icp.hasMap())
    {
        icp.setMap(mapCloud);
    }
    mapPublisher.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>(mapCloud, "world", ros::Time::now()));

    // 1
    ifstream ifs(inputFilterYamlName.c_str());
    if (ifs.good())
    {
        inputFilters = PM::DataPointsFilters(ifs);
    }

    // 2
    ifstream ifss(icpYamlName.c_str());
    if (ifss.good())
    {
        icp.loadFromYaml(ifss);
    }

    cloud_sub1 = n.subscribe("/velodyne1/velodyne_points", 1, &laser_reg::gotCloud1, this);
    cloud_sub2 = n.subscribe("/velodyne2/velodyne_points", 1, &laser_reg::gotCloud2, this);

}

void laser_reg::gotCloud1(const sensor_msgs::PointCloud2& cloudMsgIn)
{
    this->laserCloud1 = DP(PointMatcher_ros::rosMsgToPointMatcherCloud<float>(cloudMsgIn));

    this->T_laser12 = PointMatcher_ros::eigenMatrixToDim<float>(
               PointMatcher_ros::transformListenerToEigenMatrix<float>(
               this->tf_listener_calib_lasers,
               "laser1",
               "laser2",
               cloudMsgIn.header.stamp
           ), laserCloud1.getHomogeneousDim());

    laserCloud1 = transformation->compute(laserCloud1, this->T_laser12.inverse());

}

void laser_reg::gotCloud2(const sensor_msgs::PointCloud2& cloudMsgIn)
{
    if(laserCloud1.features.cols() == 0)
    {
        return;
    }

    this->laserCloud2 = DP(PointMatcher_ros::rosMsgToPointMatcherCloud<float>(cloudMsgIn));

    conCloud = laserCloud2;
    conCloud.concatenate(laserCloud1);

    // filter the input cloud
    inputFilters.apply(conCloud);

    ///do icp
    this->registration(conCloud, cloudMsgIn.header.stamp);
}

void laser_reg::registration(DP cloudIn, const ros::Time& stamp)
{

    cout<<"-------------------!!!!-------------------"<<endl;

    ROS_INFO_STREAM("input points' num:  " << cloudIn.features.cols());

    this->T_base2world = PointMatcher_ros::eigenMatrixToDim<float>(
               PointMatcher_ros::transformListenerToEigenMatrix<float>(
               this->tf_listener_base2world,
               "world",
               "base_footprint",
               stamp
           ), cloudIn.features.rows());

    this->T_laser22base = PointMatcher_ros::eigenMatrixToDim<float>(
               PointMatcher_ros::transformListenerToEigenMatrix<float>(
               this->tf_listener_calib_laser2basefootprint,
               "base_footprint",
               "laser2",
               stamp
           ), cloudIn.features.rows());

    T_laser22world = T_laser22base * T_base2world;

    PM::TransformationParameters T_laser22world_new = icp(cloudIn, T_laser22world);

    ROS_INFO_STREAM("icp over_lap:  " << icp.errorMinimizer->getOverlap());

    PM::TransformationParameters T_base2world_new = T_laser22base.inverse() * T_laser22world_new;

    tf_broader_base2world.sendTransform(PointMatcher_ros::eigenMatrixToStampedTransform<float>(T_base2world_new, "world", "base_footprint", stamp));

}

int main(int argc, char **argv)
{

    // INIT
    ros::init(argc, argv, "laser_reg");
    ros::NodeHandle n;

    laser_reg laser_reg_(n);

    ros::MultiThreadedSpinner spinner(1);
    spinner.spin();

}
