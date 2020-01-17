#include <ros/ros.h>
// PCL specific includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <opencv2/core/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <fstream>
#include <sstream>

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

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
    int startNum = 31;
	int goalNum = 63;
    std::string saveDir = "/home/nakano-lab/robomani/";
    std::string mapPath = "/mnt/d/map.pcd";
    pn.getParam("floorUnder", floorUnder);
    pn.getParam("floorUpper", floorUpper);
    pn.getParam("wallUpper", wallUpper);
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
    pn.getParam("saveDir", saveDir);
    pn.getParam("startNum", startNum);
	pn.getParam("goalNum", goalNum);

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

    pcl::PointXYZ minPt, maxPt;
    pcl::getMinMax3D (*floorCloud, minPt, maxPt);
    ROS_INFO("minY:%f", minPt.y);
    ROS_INFO("maxX:%f", maxPt.x);

    std::ofstream ofs(saveDir+"param.txt");
    ofs << std::to_string(mapCols) << std::endl;
    ofs << std::to_string(mapRows) << std::endl;
    ofs << std::to_string(unitRange) << std::endl;
    ofs << std::to_string(x0) << std::endl;
    ofs << std::to_string(y0) << std::endl;
    ofs << std::to_string(minPt.y) << std::endl;
    ofs << std::to_string(maxPt.x) << std::endl;
    ofs.close();

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
    // load track file
    ifstream ifs(saveDir+"track.txt");
    std::string line;
    std::vector<float> vec, trackX, trackY;
    while(std::getline(ifs, line)){
        std::istringstream stream(line);
        std::string str;
        while(std::getline(stream, str, ',')) vec.push_back(std::stof(str));
    }
    for(int i=0; i<vec.size(); i++){
        if(i%2==0)trackX.push_back(vec.at(i));
        else trackY.push_back(vec.at(i));
    }
    ifs.close();
    for(int i=0; i<trackX.size(); i++){
        int x = std::round(((mapCols-unitRange-x0-1)/minPt.y)*trackY.at(i) + x0);
        int y = std::round(((mapRows-y0-1)/maxPt.x)*trackX.at(i) + y0);
        if(x<mapCols&&y<mapRows)mapImage.data[y * mapImage.step + x * mapImage.elemSize()] = 255;
        if(x+1<mapCols&&y<mapRows)mapImage.data[y * mapImage.step + (x+1) * mapImage.elemSize()] = 255;
        if(x<mapCols&&y+1<mapRows)mapImage.data[(y+1) * mapImage.step + x * mapImage.elemSize()] = 255;
        if(x-1<mapCols&&y<mapRows)mapImage.data[y * mapImage.step + (x-1) * mapImage.elemSize()] = 255;
        if(x<mapCols&&y-1<mapRows)mapImage.data[(y-1) * mapImage.step + x * mapImage.elemSize()] = 255;
        if(x+1<mapCols&&y+1<mapRows)mapImage.data[(y+1) * mapImage.step + (x+1) * mapImage.elemSize()] = 255;
        if(x-1<mapCols&&y+1<mapRows)mapImage.data[(y+1) * mapImage.step + (x-1) * mapImage.elemSize()] = 255;
        if(x+1<mapCols&&y-1<mapRows)mapImage.data[(y-1) * mapImage.step + (x+1) * mapImage.elemSize()] = 255;
        if(x-1<mapCols&&y-1<mapRows)mapImage.data[(y-1) * mapImage.step + (x-1) * mapImage.elemSize()] = 255;
        ROS_INFO("unko");
    }
    cv::imwrite(saveDir+"map.jpg", mapImage);// openCV visualizing

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
    cv::imwrite(saveDir+"map_wall_comp.jpg", mapImage);
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
    cv::imwrite(saveDir+"map_wall_floor_comp.jpg", mapImage);
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
	cv::imwrite(saveDir+"map_wall_floor_wall_comp.jpg", mapImage);

    // final complement and make start and goal
    for(int x=0; x<mapCols; x++){
        for(int y=0; y<mapRows; y++){
            if(mapImage.data[ y * mapImage.step + x * mapImage.elemSize()]==127)mapImage.data[ y * mapImage.step + x * mapImage.elemSize()]=0;
        }
    }
    int startX = std::round(x0);
    int startY = std::round(y0);
    int goalX = std::round(((mapCols-unitRange-x0-1)/minPt.y)*trackY.at(trackY.size()-1) + x0);
    int goalY = std::round(((mapRows-y0-1)/maxPt.x)*trackX.at(trackX.size()-1)  + y0);
    mapImage.data[ startY * mapImage.step + startX * mapImage.elemSize()] = startNum;
    mapImage.data[ goalY * mapImage.step + goalX * mapImage.elemSize()] = goalNum;
    cv::imwrite(saveDir+"map_comp.png", mapImage);

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
