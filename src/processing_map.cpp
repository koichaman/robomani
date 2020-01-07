#include <ros/ros.h>
// PCL specific includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
// #include <pcl/ModelCoefficients.h>
// #include <pcl/filters/extract_indices.h>
// #include <pcl/sample_consensus/method_types.h>
// #include <pcl/sample_consensus/model_types.h>
// #include <pcl/segmentation/sac_segmentation.h>
#include <opencv2/core/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/highgui/highgui.hpp>

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

// pcl::PointCloud<pcl::PointXYZ>::Ptr planeExtract(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double threshold){
// 	//　平面検出（pclのチュートリアル通り）
// 	pcl::SACSegmentation<pcl::PointXYZ> seg;
// 	seg.setOptimizeCoefficients(true);
// 	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
// 	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
// 	seg.setInputCloud(cloud);
// 	seg.setModelType(pcl::SACMODEL_PLANE); //モデル
// 	seg.setMethodType(pcl::SAC_RANSAC);	//検出手法
// 	seg.setMaxIterations(200);
// 	seg.setDistanceThreshold(threshold); //閾値
// 	seg.segment(*inliers, *coefficients);
//
// 	// extract points on the detected plane
// 	pcl::ExtractIndices<pcl::PointXYZ> extract;
// 	extract.setInputCloud(cloud);
// 	extract.setIndices(inliers);
// 	pcl::PointCloud<pcl::PointXYZ>::Ptr result(new pcl::PointCloud<pcl::PointXYZ>);
// 	extract.filter(*result);
//
// 	return result;
// }

int main (int argc, char** argv){
    // Initialize ROS
    ros::init (argc, argv, "making_map");
    ros::NodeHandle nh;
    ros::NodeHandle pn("~");
    float floorUnder = -2.1;
    float floorUpper = -1;
    float wallUpper = 1;
	// float threshold = 2;
    float xRangeMax = 150;
    float yRangeMin = -150;
    int mapCols = 128;
    int mapRows = 128;
    float x0 = 0;
    float y0 = 0;
    float unitRange = 8.0;
    int wallCompNum = 4;
    int floorCompNum = 8;
    std::string mapPath = "/mnt/d/map.pcd";
    pn.getParam("floorUnder", floorUnder);
    pn.getParam("floorUpper", floorUpper);
    pn.getParam("wallUpper", wallUpper);
	// pn.getParam("threshold", threshold);
    pn.getParam("xRangeMax", xRangeMax);
    pn.getParam("yRangeMin", yRangeMin);
    pn.getParam("mapCols", mapCols);
    pn.getParam("mapRows", mapRows);
    pn.getParam("mapPath", mapPath);
    pn.getParam("x0", x0);
    pn.getParam("y0", y0);
    pn.getParam("unitRange", unitRange);
    pn.getParam("wallCompNum", wallCompNum);
    pn.getParam("floorCompNum", floorCompNum);

    // Initialize openCV valiable
    cv::Mat mapImage(mapCols, mapRows, CV_MAKETYPE(CV_8U, 1));
    for(int x=0; x<mapCols; x++){
        for(int y=0; y<mapRows; y++) mapImage.data[ y * mapImage.step + x * mapImage.elemSize()] = 127;
    }

    // load pcd data
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (mapPath, *cloud) == -1){
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
        return (-1);
    }
    // make instance of PCL viewer
    pcl::visualization::PCLVisualizer viewer("PointCloudViewer");
    ros::spinOnce();
    viewer.addPointCloud(cloud);
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2);
    viewer.addCoordinateSystem(10);
    viewer.setCameraPosition(-5,0,0,0,0,0,0,0,1);
    viewer.spinOnce();
    ros::Rate r(10); // 10 hz
    while(ros::ok()){
        ros::spinOnce();
        viewer.updatePointCloud(cloud);
	    viewer.spinOnce();
        r.sleep();
        if(viewer.wasStopped())break;
    }

    // apply filter to detect the floor
    ros::spinOnce();
    pcl::PointCloud<pcl::PointXYZ>::Ptr intermidCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr floorCloud(new pcl::PointCloud<pcl::PointXYZ>);
    // Create the filtering object
    pcl::PassThrough<pcl::PointXYZ> floorDetectFilter;
    // x
    floorDetectFilter.setInputCloud (cloud);
    floorDetectFilter.setFilterFieldName ("x");
    floorDetectFilter.setFilterLimits (-1000, xRangeMax);
    floorDetectFilter.filter (*intermidCloud);
    // y
    floorDetectFilter.setInputCloud (intermidCloud);
    floorDetectFilter.setFilterFieldName ("y");
    floorDetectFilter.setFilterLimits (yRangeMin, 1000);
    floorDetectFilter.filter (*intermidCloud);
	// pcl visualizing
    viewer.removeAllPointClouds();
    viewer.removeAllShapes();
    viewer.addPointCloud(intermidCloud);
    viewer.spinOnce();
    while(ros::ok()){
        ros::spinOnce();
        r.sleep();
        viewer.updatePointCloud(intermidCloud);
        viewer.spinOnce();
        if(viewer.wasStopped())break;
    }
    // z
    floorDetectFilter.setFilterFieldName ("z");
    floorDetectFilter.setFilterLimits (floorUnder, floorUpper);
    floorDetectFilter.filter (*floorCloud);
	// floorCloud = planeExtract(intermidCloud, threshold);

    pcl::PointXYZ minPt, maxPt;
    pcl::getMinMax3D (*floorCloud, minPt, maxPt);
    ROS_INFO("minY:%f", minPt.y);
    ROS_INFO("maxX:%f", maxPt.x);
    // convert to openCV image
    for(int i=0; i<floorCloud->points.size(); i++){
		pcl::PointXYZ &point = floorCloud->points[i];
        int x = std::round(((mapCols-unitRange-x0-1)/minPt.y)*point.y + x0);
        int y = std::round(((mapRows-y0-1)/maxPt.x)*point.x + y0);
        if(x>=mapCols)x=mapCols-1;
        else if(x<0)x=0;
        if(y>=mapRows)y=mapRows-1;
        else if(y<0)y=0;
        mapImage.data[y * mapImage.step + x * mapImage.elemSize()] = 255;
    }

    // apply filter to detect the walls
    pcl::PointCloud<pcl::PointXYZ>::Ptr wallCloud(new pcl::PointCloud<pcl::PointXYZ>);
    // Create the filtering object
    pcl::PassThrough<pcl::PointXYZ> wallDetectFilter;
    // x
    wallDetectFilter.setInputCloud (intermidCloud);
    wallDetectFilter.setFilterFieldName ("z");
    wallDetectFilter.setFilterLimits (floorUpper, wallUpper);
    wallDetectFilter.filter (*wallCloud);
    // convert to openCV image
    for(int i=0; i<wallCloud->points.size(); i++){
		pcl::PointXYZ &point = wallCloud->points[i];
        int x = std::round(((mapCols-unitRange-x0-1)/minPt.y)*point.y + x0);
        int y = std::round(((mapRows-y0-1)/maxPt.x)*point.x + y0);
        if(x>=mapCols)x=mapCols-1;
        else if(x<0)x=0;
        if(y>=mapRows)y=mapRows-1;
        else if(y<0)y=0;
        mapImage.data[y * mapImage.step + x * mapImage.elemSize()] = 0;
    }
    cv::imwrite("/mnt/d/map.jpg", mapImage);// openCV visualizing

    // complement walls
    bool needComp = false;
	bool keepLoop = true;
    while(keepLoop){
		keepLoop = false;
        for(int x=0; x<mapCols-wallCompNum; x++){
            for(int y=0; y<mapRows-wallCompNum; y++){
                needComp = false;
                if(mapImage.data[y*mapImage.step + x*mapImage.elemSize()]==0){
                    int i;
                    for(i=1; i<=wallCompNum; i++){
                        if(mapImage.data[y*mapImage.step + (x+i)*mapImage.elemSize()]==0){
							needComp=true;
							if(i>1)keepLoop = true;
							break;
						}
                    }
                    if(needComp){
                        for(int j=1; j<i; j++){
                            mapImage.data[y*mapImage.step + (x+j)*mapImage.elemSize()] = 0;
                        }
                    }
                    needComp = false;
                    for(i=1; i<=wallCompNum; i++){
                        if(mapImage.data[(y+i)*mapImage.step + x*mapImage.elemSize()]==0){
							needComp=true;
							if(i>1)keepLoop = true;
							break;
						}
                    }
                    if(needComp){
                        for(int j=1; j<i; j++){
                            mapImage.data[(y+j)*mapImage.step + x*mapImage.elemSize()] = 0;
                        }
                    }
                }
            }
        }
    }
    cv::imwrite("/mnt/d/map_wall_comp.jpg", mapImage);
    // complement floor
	keepLoop = true;
	while(keepLoop){
		keepLoop = false;
	    for(int x=floorCompNum; x<mapCols-floorCompNum; x++){
	        for(int y=floorCompNum; y<mapRows-floorCompNum; y++){
	            needComp = false;
	            if(mapImage.data[y*mapImage.step + x*mapImage.elemSize()]==255){
	                int i;
	                for(i=1; i<=floorCompNum; i++){
	                    if(mapImage.data[y*mapImage.step + (x+i)*mapImage.elemSize()]%255==0){
							needComp=true;
							if(i>1)keepLoop = true;
							break;
						}
	                }
	                if(needComp){
	                    for(int j=1; j<i; j++){
	                        mapImage.data[y*mapImage.step + (x+j)*mapImage.elemSize()] = 255;
	                    }
	                }
	                needComp = false;
	                for(i=1; i<=floorCompNum; i++){
	                    if(mapImage.data[(y+i)*mapImage.step + x*mapImage.elemSize()]%255==0){
							needComp=true;
							if(i>1)keepLoop = true;
							break;
						}
	                }
	                if(needComp){
	                    for(int j=1; j<i; j++){
	                        mapImage.data[(y+j)*mapImage.step + x*mapImage.elemSize()] = 255;
	                    }
	                }
					needComp = false;
	                for(i=1; i<=floorCompNum; i++){
	                    if(mapImage.data[y*mapImage.step + (x-i)*mapImage.elemSize()]%255==0){
							needComp=true;
							if(i>1)keepLoop = true;
	                    	break;
						}
	                }
	                if(needComp){
	                    for(int j=1; j<i; j++){
	                        mapImage.data[y*mapImage.step + (x-j)*mapImage.elemSize()] = 255;
	                    }
	                }
	                needComp = false;
	                for(i=1; i<=floorCompNum; i++){
	                    if(mapImage.data[(y-i)*mapImage.step + x*mapImage.elemSize()]%255==0){
							needComp=true;
							if(i>1)keepLoop = true;
							break;
						}
	                }
	                if(needComp){
	                    for(int j=1; j<i; j++){
	                        mapImage.data[(y-j)*mapImage.step + x*mapImage.elemSize()] = 255;
	                    }
	                }
	            }
	        }
	    }
	}
    cv::imwrite("/mnt/d/map_wall_floor_comp.jpg", mapImage);
	// complement wall 2
	keepLoop = false;
	for(int x=0; x<mapCols-unitRange*1.5; x++){
		for(int y=0; y<mapRows-unitRange*1.5; y++){
			needComp = false;
			if(mapImage.data[y*mapImage.step + x*mapImage.elemSize()]==0){
				int i;
				for(i=1; i<=unitRange*1.5; i++){
					if(mapImage.data[y*mapImage.step + (x+i)*mapImage.elemSize()]==255) break;
					else if(mapImage.data[y*mapImage.step + (x+i)*mapImage.elemSize()]==0){
						needComp=true;
						break;
					}
				}
				if(needComp){
					for(int j=1; j<i; j++){
						mapImage.data[y*mapImage.step + (x+j)*mapImage.elemSize()] = 0;
					}
				}
				needComp = false;
				for(i=1; i<=unitRange*1.5; i++){
					if(mapImage.data[(y+i)*mapImage.step + x*mapImage.elemSize()]==255) break;
					else if(mapImage.data[(y+i)*mapImage.step + x*mapImage.elemSize()]==0){
						needComp=true;
						break;
					}
				}
				if(needComp){
					for(int j=1; j<i; j++){
						mapImage.data[(y+j)*mapImage.step + x*mapImage.elemSize()] = 0;
					}
				}
			}
		}
	}
	cv::imwrite("/mnt/d/map_wall_floor_wall_comp.jpg", mapImage);

    // pcl visualizing
    viewer.removeAllPointClouds();
    viewer.removeAllShapes();
    viewer.addPointCloud(floorCloud);
    viewer.spinOnce();
    while(ros::ok()){
        ros::spinOnce();
        r.sleep();
        viewer.updatePointCloud(floorCloud);
        viewer.spinOnce();
        if(viewer.wasStopped())break;
    }
}
