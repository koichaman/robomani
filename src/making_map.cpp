#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>

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
}

int main (int argc, char** argv){
    // Initialize ROS
    ros::init (argc, argv, "making_map");
    ros::NodeHandle nh;
    ros::NodeHandle pn("~");
    float cameraHeight = 0.2;
    float floorRange = 0.01;
    pn.getParam("cameraHeight",cameraHeight);
    pn.getParam("floorRange",floorRange);
    // make instance of PCL viewer
    pcl::visualization::PCLVisualizer viewer("PointCloudViewer");

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber subPCloud = nh.subscribe ("/orb_slam2/map_points", 1, cloud_cb);
    ros::Subscriber subPose = nh.subscribe ("/orb_slam2/pose", 1, pose_cb);

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
        viewer.removeShape("arrow");
        viewer.addArrow(pclCameraArrowPt, cameraPt, 0, 1, 0, false);
        viewer.updatePointCloud(cloud);
        viewer.updateSphere(cameraPt, SPHERE_RAD, 0, 1, 0);
	    viewer.spinOnce();
        r.sleep();
        if(viewer.wasStopped())break;
    }
    ros::spinOnce();
    pcl::io::savePCDFileASCII ("/mnt/d/map.pcd" , *cloud);
    while(ros::ok()){
        ros::spinOnce();
        r.sleep();
        viewer.updatePointCloud(cloud);
        viewer.spinOnce();
        if(viewer.wasStopped())break;
    }
}
