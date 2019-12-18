#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#define SPHERE_RAD 0.1
#define ARROW_SCALE 1.0

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointXYZ cameraPt(0,0,0);
tf::Transform cameraTf;
tf::Vector3 cameraDir(1,0,0);
tf::Vector3 UNIT_VECTOR_X(1,0,0);
pcl::PointXYZ cameraArrowPt(0,0,0);

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
    cameraDir = cameraTf * UNIT_VECTOR_X;
    cameraArrowPt.x = cameraPt.x + ARROW_SCALE * cameraDir.getX();
    cameraArrowPt.y = cameraPt.y + ARROW_SCALE * cameraDir.getY();
    cameraArrowPt.z = cameraPt.z + ARROW_SCALE * cameraDir.getZ();
}

int main (int argc, char** argv){
    // Initialize ROS
    ros::init (argc, argv, "making_map");
    ros::NodeHandle nh;
    // make instance of PCL viewer
    pcl::visualization::PCLVisualizer viewer("PointCloudViewer");

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber subPCloud = nh.subscribe ("/orb_slam2_mono/map_points", 1, cloud_cb);
    ros::Subscriber subPose = nh.subscribe ("/orb_slam2_mono/pose", 1, pose_cb);

    ros::spinOnce();
    viewer.addPointCloud(cloud);
    viewer.addArrow(cameraArrowPt, cameraPt, 0, 1, 0, false);
    viewer.addSphere(cameraPt, SPHERE_RAD, 0, 1, 0);
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2);
    viewer.addCoordinateSystem();
    viewer.setCameraPosition(-20, 0, 5, 0, 0, 0, 1, 0, 3);
    viewer.spinOnce();

    ros::Rate r(10); // 10 hz
    while(ros::ok()){
        ros::spinOnce();
        viewer.removeShape("arrow");
        viewer.addArrow(cameraArrowPt, cameraPt, 0, 1, 0, false);
        viewer.updatePointCloud(cloud);
        viewer.updateSphere(cameraPt, SPHERE_RAD, 0, 1, 0);
	    viewer.spinOnce();
        r.sleep();
    }
}
