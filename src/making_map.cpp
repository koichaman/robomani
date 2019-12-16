#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input){
    // Create a container for the data.
    sensor_msgs::PointCloud2 output;
    // convert from ROS msg class to PCL class
    pcl::fromROSMsg (*input, *cloud);
}

int main (int argc, char** argv){
    // Initialize ROS
    ros::init (argc, argv, "making_map");
    ros::NodeHandle nh;
    // make instance of PCL viewer
    pcl::visualization::PCLVisualizer viewer("PointCloudViewer");

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe ("/orb_slam2_mono/map_points", 1, cloud_cb);

    ros::spinOnce();
    viewer.addPointCloud(cloud);
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2);
    viewer.addCoordinateSystem();
    // viewer.setCameraPosition(-30, 0, 10, 0, 0, 0, 1, 0, 3);
    viewer.spinOnce();

    ros::Rate r(10); // 10 hz
    while(ros::ok()){
        ros::spinOnce();
        viewer.updatePointCloud(cloud);
	    viewer.spinOnce();
        r.sleep();
    }
    // test commit
}
