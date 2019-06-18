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

class loc_demo
{

    typedef PointMatcher<float> PM;
    typedef PM::DataPoints DP;

public:
    loc_demo(ros::NodeHandle &n);
    ~loc_demo();
    ros::NodeHandle& n;



};

loc_demo::~loc_demo()
{}

loc_demo::loc_demo(ros::NodeHandle& n):
    n(n)
{

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
