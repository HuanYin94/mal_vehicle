#include "ros/ros.h"
#include "ros/console.h"

#include "eigen_conversions/eigen_msg.h"

#include <tf/transform_broadcaster.h>

#include "ros/publisher.h"
#include "ros/subscriber.h"

#include "pointmatcher_ros/transform.h"
#include "pointmatcher_ros/point_cloud.h"
#include "nabo/nabo.h"
#include "pointmatcher_ros/get_params_from_server.h"

#include <fstream>
#include <visualization_msgs/Marker.h>

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
    PM::TransformationParameters trans;
    unique_ptr<PM::Transformation> transformation;

    tf::TransformListener tf_listener_calib;
    tf::TransformListener tf_listener_base2world;

    string inputFilterYamlName;
    PM::DataPointsFilters inputFilters;

    PM::TransformationParameters registration(DP cloudIn);

    PM::ICPSequence icp;
    PM::TransformationParameters T_base2world, T_base2world_new;


};

laser_reg::~laser_reg()
{}

laser_reg::laser_reg(ros::NodeHandle& n):
    n(n),
    loadMapName(getParam<string>("loadMapName", ".")),
    transformation(PM::get().REG(Transformation).create("RigidTransformation")),
    inputFilterYamlName(getParam<string>("inputFilterYamlName", "."))
{
    // prepare
    mapCloud = DP::load(loadMapName);
    if(!icp.hasMap())
    {
        icp.setMap(mapCloud);
    }

    ifstream ifs(inputFilterYamlName.c_str());
    if (ifs.good())
    {
        inputFilters = PM::DataPointsFilters(ifs);
    }

    cloud_sub1 = n.subscribe("/velodyne1/velodyne_points", 1, &laser_reg::gotCloud1, this);
    cloud_sub2 = n.subscribe("/velodyne2/velodyne_points", 1, &laser_reg::gotCloud2, this);

}

void laser_reg::gotCloud1(const sensor_msgs::PointCloud2& cloudMsgIn)
{
    this->laserCloud1 = DP(PointMatcher_ros::rosMsgToPointMatcherCloud<float>(cloudMsgIn));

    this->trans = PointMatcher_ros::eigenMatrixToDim<float>(
               PointMatcher_ros::transformListenerToEigenMatrix<float>(
               this->tf_listener_calib,
               "laser1",
               "laser2",
               cloudMsgIn.header.stamp
           ), laserCloud1.getHomogeneousDim());

    laserCloud1 = transformation->compute(laserCloud1, this->trans.inverse());

}

void laser_reg::gotCloud2(const sensor_msgs::PointCloud2& cloudMsgIn)
{
    if(laserCloud1.features.cols() == 0)
        return;

    this->laserCloud2 = DP(PointMatcher_ros::rosMsgToPointMatcherCloud<float>(cloudMsgIn));

    conCloud = laserCloud2;
    conCloud.concatenate(laserCloud1);

    inputFilters.apply(conCloud);

    ///do icp
    PM::TransformationParameters test = this->registration(conCloud);
}

laser_reg::PM::TransformationParameters laser_reg::registration(DP cloudIn)
{

    T_base2world_new = icp(*cloudIn, T_base2world);


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
