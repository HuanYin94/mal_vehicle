#include "ros/ros.h"
#include "ros/console.h"

#include "eigen_conversions/eigen_msg.h"

#include <fstream>
#include <vector>
#include <time.h>
#include <math.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>

#include "tf/transform_listener.h"
#include "geometry_msgs/Pose2D.h"

#define PORT    8844
#define MAXLINE 1024
#define BYTENUM 30 // 16 + 4*3 + 2
#define SLEEPSEC 50000 //20Hz // 10e6 = 1s

using namespace std;

struct header
{
    unsigned short int flag;
    unsigned short int type;
    unsigned short int ID;
    unsigned short int length;
    unsigned int sec;
    unsigned int miliSec;
};

struct udpMsg
{
    struct header Header;
    float pose_x;
    float pose_y;
    float heading;
    unsigned short int status;
};

int sockfd;
char buffer[MAXLINE];
struct sockaddr_in servaddr;

unsigned short int idCnt = 0x0000;

void senderCallback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
    udpMsg toVeh;

    toVeh.Header.flag = 0xcece;
    toVeh.Header.type = 0x0001;
    toVeh.Header.ID = idCnt;
    toVeh.Header.length = 16;
    toVeh.Header.sec = ros::Time::now().toSec();
    toVeh.Header.miliSec = ros::Time::now().toNSec();

    toVeh.pose_x = msg->x;
    toVeh.pose_y = msg->y;
    toVeh.heading = msg->theta;
    toVeh.status = 1;

    memcpy(buffer, &toVeh, sizeof(toVeh));

    sendto(sockfd, (const char *)buffer, BYTENUM,
        MSG_CONFIRM, (const struct sockaddr *) &servaddr,
            sizeof(servaddr));

    cout<<"-----------------------------------------------"<<endl;

    cout<<toVeh.Header.ID <<"  "<<
          toVeh.pose_x<<"  "<<
          toVeh.pose_y<<"  "<<
          toVeh.heading<<endl;

    idCnt = idCnt + 1;
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "sender_udp");
    ros::NodeHandle n;

    // tf & transform

    // Creating socket file descriptor
    if ( (sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) {
        perror("socket creation failed");
        exit(EXIT_FAILURE);
    }

    memset(&servaddr, 0, sizeof(servaddr));

    // Filling server information
    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(PORT);
    servaddr.sin_addr.s_addr = inet_addr("192.168.1.12");

    while+(n.ok())
    {

    }


//    ros::Subscriber sub = n.subscribe("veh_sta_udp", 1, senderCallback);



    ros::spin();

    return 0;
}
