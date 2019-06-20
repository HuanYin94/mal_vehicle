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
using namespace PointMatcherSupport;

class map_filter
{

    typedef PointMatcher<float> PM;
    typedef PM::DataPoints DP;

public:
    map_filter(ros::NodeHandle &n);
    ~map_filter();
    ros::NodeHandle& n;

    string loadMapName, saveMapName;
    string filterYamlName;

    PM::DataPointsFilters mapFilters;

    int intTh;
};

map_filter::~map_filter()
{}

map_filter::map_filter(ros::NodeHandle& n):
    n(n),
    loadMapName(getParam<string>("loadMapName", ".")),
    saveMapName(getParam<string>("saveMapName", ".")),
    filterYamlName(getParam<string>("filterYamlName", ".")),
    intTh(getParam<int>("intTh", 0))
{
    DP mapCloud = DP::load(loadMapName);

    ifstream ifs(filterYamlName.c_str());
    if (ifs.good())
    {
        mapFilters = PM::DataPointsFilters(ifs);
    }

    mapFilters.apply(mapCloud);

    /// intensity filter ?

    DP mapCloud_new = mapCloud.createSimilarEmpty();
    int cnt = 0;
    int rowLine = mapCloud.getDescriptorStartingRow("intensity");
    for(int i=0; i<mapCloud.features.cols(); i++)
    {
        if(mapCloud.descriptors(rowLine, i) > this->intTh)
        {
            mapCloud_new.setColFrom(cnt, mapCloud, i);
            cnt++;
        }
    }
    mapCloud_new.conservativeResize(cnt);

    mapCloud_new.save(saveMapName);
}

int main(int argc, char **argv)
{

    // INIT
    ros::init(argc, argv, "map_filter");
    ros::NodeHandle n;

    map_filter map_filter_(n);

    ros::MultiThreadedSpinner spinner(1);
    spinner.spin();

}
