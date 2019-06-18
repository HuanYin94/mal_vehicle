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

class map_transformer
{

    typedef PointMatcher<float> PM;
    typedef PM::DataPoints DP;

public:
    map_transformer(ros::NodeHandle &n);
    ~map_transformer();
    ros::NodeHandle& n;

    float x1, y1, z1, yaw1;  // laser to vehicle center
    float x2, y2, yaw2;  // vehicle center to mag world

    string loadMapName, saveMapName;
    PM::TransformationParameters PMtrans1, PMtrans2;
    unique_ptr<PM::Transformation> transformation;

};

map_transformer::~map_transformer()
{}

map_transformer::map_transformer(ros::NodeHandle& n):
    n(n),
    transformation(PM::get().REG(Transformation).create("RigidTransformation")),
    x1(getParam<float>("x1", 0)),
    y1(getParam<float>("y1", 0)),
    z1(getParam<float>("z1", 0)),
    yaw1(getParam<float>("yaw1", 0)),
    x2(getParam<float>("x2", 0)),
    y2(getParam<float>("y2", 0)),
    yaw2(getParam<float>("yaw2", 0)),
    loadMapName(getParam<string>("loadMapName", ".")),
    saveMapName(getParam<string>("saveMapName", "."))
{
    DP mapCloud = DP::load(loadMapName);

    // magic

    AngleAxisf V1(yaw1, Vector3f(0, 0, 1));
    Matrix3f rot1;
    rot1 = V1.toRotationMatrix();
    Matrix4f trans1;
    trans1 << rot1(0,0), rot1(0,1), rot1(0,2), x1,
               rot1(1,0), rot1(1,1), rot1(1,2), y1,
               rot1(2,0), rot1(2,1), rot1(2,2), z1,
               0, 0, 0, 1;
    PMtrans1 = PointMatcher_ros::eigenMatrixToDim<float>(trans1, 4);

    AngleAxisf V2(yaw2, Vector3f(0, 0, 1));
    Matrix3f rot2;
    rot2 = V2.toRotationMatrix();
    Matrix4f trans2;
    trans2 << rot2(0,0), rot2(0,1), rot2(0,2), x2,
               rot2(1,0), rot2(1,1), rot2(1,2), y2,
               rot2(2,0), rot2(2,1), rot2(2,2), 0,
               0, 0, 0, 1;
    PMtrans2 = PointMatcher_ros::eigenMatrixToDim<float>(trans2, 4);

    DP mapCloud_new = transformation->compute(mapCloud, PMtrans1);
    DP mapCloud_new_new = transformation->compute(mapCloud_new, PMtrans2);

    mapCloud_new_new.save(saveMapName);

}

int main(int argc, char **argv)
{

    // INIT
    ros::init(argc, argv, "map_transformer");
    ros::NodeHandle n;

    map_transformer map_transformer_(n);

    ros::MultiThreadedSpinner spinner(1);
    spinner.spin();

}
