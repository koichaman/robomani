#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Float32MultiArray.h>
// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <time.h>

#define SPHERE_RAD 0.1
#define ARROW_SCALE 1.0
#define BACK_SCALE 3.0
#define CAMERA_ADDITINAL_Z 1.0

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointXYZ cameraPt(0,0,0);
tf::Transform cameraTf;
tf::Vector3 cameraViewDir(1,0,0);
tf::Vector3 cameraUpDir(0,0,1);
tf::Vector3 UNIT_VECTOR_X(1,0,0);
tf::Vector3 UNIT_VECTOR_Z(0,0,1);
tf::Vector3 tfCameraArrowPt(0,0,0);
pcl::PointXYZ pclCameraArrowPt(0,0,0);
// params
int mapCols, mapRows;
float unitRange, initX, initY, minY, maxX;
// subscribe valiables
float x, y, dirX, dirY, vel, acc, lastX, lastY, lastVel;
clock_t last = 0;
clock_t now;

pcl::PointXYZ tfVectorToPCL(tf::Vector3 tfVec){
    pcl::PointXYZ output;
    output.x = tfVec.getX();
    output.y = tfVec.getY();
    output.z = tfVec.getZ();
    return output;
}

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input){
    // Create a container for the data.
    sensor_msgs::PointCloud2 output;
    // convert from ROS msg class to PCL class
    pcl::fromROSMsg (*input, *cloud);
}

void pose_cb (const geometry_msgs::PoseStamped& msg){
    poseMsgToTF(msg.pose, cameraTf);
    cameraPt.x = cameraTf.getOrigin().getX();
    cameraPt.y = cameraTf.getOrigin().getY();
    cameraPt.z = cameraTf.getOrigin().getZ();
    tfCameraArrowPt = cameraTf * (ARROW_SCALE * UNIT_VECTOR_X);
    pclCameraArrowPt = tfVectorToPCL(tfCameraArrowPt);
    cameraViewDir = (1/ARROW_SCALE) * (tfCameraArrowPt - cameraTf.getOrigin());
    cameraUpDir = cameraTf * UNIT_VECTOR_Z - cameraTf.getOrigin();
    x = ((mapCols-unitRange-initX-1)/minY)*cameraPt.y + initX;
    y = ((mapRows-initY-1)/maxX)*cameraPt.x + initY;
    float convertedArrowPtX, convertedArrowPtY;
    convertedArrowPtX = ((mapCols-unitRange-initX-1)/minY)*pclCameraArrowPt.y + initX;
    convertedArrowPtY = ((mapRows-initY-1)/maxX)*pclCameraArrowPt.x + initY;
    dirX = convertedArrowPtX - x;
    dirY = convertedArrowPtY - y;
    float norm = std::sqrt(dirX*dirX + dirY*dirY);
    dirX /= norm;
    dirY /= norm;
    now = clock();
    if(last!=0){
        float dt = (now-last)/CLOCKS_PER_SEC;
        vel = std::sqrt((x-lastX)*(x-lastX)+(y-lastY)*(y-lastY))/dt;
        acc = (vel-lastVel)/dt;
    }
    last = now;
    lastX = x;
    lastY = y;
    lastVel = vel;
}

int main (int argc, char** argv){
    // Initialize ROS
    ros::init (argc, argv, "making_map");
    ros::NodeHandle nh;
    ros::NodeHandle pn("~");
    float cameraHeight = 0.2;
    float floorRange = 0.01;
    std::string paramFilePath = "/home/nakano-lab/robomani/param.txt";
    std::string pcdSavePath = "/home/nakano-lab/robomani/map.pcd";
    pn.getParam("cameraHeight",cameraHeight);
    pn.getParam("floorRange",floorRange);
    pn.getParam("paramFilePath",paramFilePath);
    // make instance of PCL viewer
    pcl::visualization::PCLVisualizer viewer("PointCloudViewer");

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber subPCloud = nh.subscribe ("/orb_slam2/map_points", 1, cloud_cb);
    ros::Subscriber subPose = nh.subscribe ("/orb_slam2/pose", 1, pose_cb);

    // create a publisher
    ros::Publisher pub = nh.advertise<std_msgs::Float32MultiArray>("pos", 10);

    // load param file
    ifstream ifs(paramFilePath);
    std::string line;
    std::vector<std::string> strvec;
    while(std::getline(ifs, line))strvec.push_back(line);
    mapCols = std::stoi(strvec.at(0));
    mapRows = std::stoi(strvec.at(1));
    unitRange = std::stof(strvec.at(2));
    initX = std::stof(strvec.at(3));
    initY = std::stof(strvec.at(4));
    minY = std::stof(strvec.at(5));
    maxX = std::stof(strvec.at(6));
    ifs.close();

    ros::spinOnce();
    viewer.addPointCloud(cloud);
    viewer.addArrow(pclCameraArrowPt, cameraPt, 0, 1, 0, false);
    viewer.addSphere(cameraPt, SPHERE_RAD, 0, 1, 0);
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2);
    viewer.addCoordinateSystem(0.1);
    viewer.setCameraPosition(64,-64,20,64,-64,0,1,0,0);
    viewer.spinOnce();

    ros::Rate r(10); // 10 hz
    while(ros::ok()){
        ros::spinOnce();
        std_msgs::Float32MultiArray array;
        array.data.resize(6);
        array.data[0] = x;
        array.data[1] = y;
        array.data[2] = dirX;
        array.data[3] = dirY;
        array.data[4] = vel;
        array.data[5] = acc;
        pub.publish(array);
        viewer.removeShape("arrow");
        viewer.addArrow(pclCameraArrowPt, cameraPt, 0, 1, 0, false);
        viewer.updatePointCloud(cloud);
        viewer.updateSphere(cameraPt, SPHERE_RAD, 0, 1, 0);
	    viewer.spinOnce();
        r.sleep();
    }
}
