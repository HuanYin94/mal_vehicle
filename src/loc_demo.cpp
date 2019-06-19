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

    ros::Subscriber mag_pose_sub;
    tf::TransformListener tf_listener_base2world;
    tf::TransformBroadcaster tf_broader_base2world;

    void gotMag(const geometry_msgs::PointStamped& magMsgIn);

    int magCnt;

    PM::TransformationParameters mag2PMTransform(geometry_msgs::PointStamped pointIn);

};

loc_demo::~loc_demo()
{}

loc_demo::loc_demo(ros::NodeHandle& n):
    n(n)
{
    magCnt = 0;

    mag_pose_sub = n.subscribe("mag_pose", 1, &loc_demo::gotMag, this);

}

void loc_demo::gotMag(const geometry_msgs::PointStamped &magMsgIn)
{
    cout<<"-------------------!!!!-------------------"<<endl;
    cout<<magMsgIn.point.x<<"   "
        <<magMsgIn.point.y<<"   "
        <<magMsgIn.point.z<<endl;

    geometry_msgs::PointStamped mag_pose = magMsgIn;

    if(magCnt <= 100 )
    {

        PM::TransformationParameters T_base2world = this->mag2PMTransform(mag_pose);

        tf_broader_base2world.sendTransform(PointMatcher_ros::eigenMatrixToStampedTransform<float>(T_base2world, "world", "base_footprint", magMsgIn.header.stamp));

        cout<<"Maging"<<endl;

    }

    magCnt ++;

}

loc_demo::PM::TransformationParameters loc_demo::mag2PMTransform(geometry_msgs::PointStamped pointIn)
{
    PM::TransformationParameters t = PM::TransformationParameters::Identity(4, 4);
    t(0,3) = pointIn.point.x; t(1,3) = pointIn.point.y;
    AngleAxisf V1(pointIn.point.z, Vector3f(0, 0, 1));
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
