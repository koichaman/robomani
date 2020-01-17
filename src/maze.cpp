#include <ros/ros.h>
#include <fstream>
#include <queue>
#include <cstdio>
#include <opencv2/core/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <fstream>
#include <sstream>
using namespace std;
cv::Mat mapImage;
int N, M, i, j, k, sx, sy, gx, gy, lenth, ii, jj;
int dr[300][300], d[300][300] = {0}, bd[300][300] = {0}, bc[300][300] = {0};
int r[100000], rx[100000], ry[100000], dx[4] = {1, 0, -1, 0}, dy[4] = {0, 1, 0, -1};
typedef pair<int, int> xy;
int com[4][4] = {{0, -1, 0, 1}, {1, 0, -1, 0}, {0, 1, 0, -1}, {-1, 0, 1, 0}};

void bfs()
{
	queue<xy> que;
	que.push(xy(sx, sy));
	d[sx][sy] = 0;

	while(que.size())
	{
		xy q = que.front();
		que.pop();

		if (q.first == gx && q.second == gy)
			break;

		for (i = 0; i < 4; i++)
		{
			int x = q.first + dx[i];
			int y = q.second + dy[i];
			if (0 <= x && x < N && 0 <= y && y < M && mapImage.data[ y*mapImage.step + x*mapImage.elemSize()] != 0 && d[x][y] == 0)
			{
				que.push(xy(x, y));
				d[x][y] = d[q.first][q.second] + 1;
				dr[x][y] = i;
			}
		}
	}
}

int main(int argc, char** argv)
{
	ros::init (argc, argv, "maze");
    ros::NodeHandle nh;
    ros::NodeHandle pn("~");
	std::string savePath = "/home/nakano-lab/robomani/";
	int startNum = 31;
	int goalNum = 63;
	pn.getParam("savePath", savePath);
	pn.getParam("startNum", startNum);
	pn.getParam("goalNum", goalNum);
	mapImage = cv::imread(savePath+"map_comp.png");
	cv::imwrite(savePath+"unko.jpg", mapImage);
	N = mapImage.cols;
	M = mapImage.rows;
	for(int x=0; x<mapImage.cols; x++){
		for(int y=0; y<mapImage.rows; y++){
			if(mapImage.data[ y*mapImage.step + x*mapImage.elemSize()]==startNum){
				sx = x;
				sy = y;
			}
			else if(mapImage.data[ y*mapImage.step + x*mapImage.elemSize()]==goalNum){
				gx = x;
				gy = y;
			}
		}
	}

	bfs();

	i = gx;
	j = gy;
	for (int k = d[gx][gy]; k > 0; k--)
	{
		r[k] = dr[i][j];
		rx[k] = i;
		ry[k] = j;
		i = i - dx[r[k]];
		j = j - dy[r[k]];
	}

	r[0] = r[1];
	lenth = 0;
	for (int k = 1; k <= d[gx][gy]; k++)
	{
		if (r[k] != r[k - 1])
		{
			if (com[r[k - 1]][r[k]] == 1)
			{
				bc[rx[k - 1]][ry[k - 1]] = 1;
			}
			else
			{
				bc[rx[k - 1]][ry[k - 1]] = 2;
			}
			ii = rx[k - 2];
			jj = ry[k - 2];
			for (i = 1; i <= lenth; i++)
			{
				bd[ii][jj] = i;
				ii = ii - dx[r[k - 1]];
				jj = jj - dy[r[k - 1]];
			}
			lenth = 0;
		}
		lenth++;
	}
	ii = rx[d[gx][gy] - 1];
	jj = ry[d[gx][gy] - 1];
	for (i = 1; i <= lenth; i++)
	{
		bd[ii][jj] = i;
		ii = ii - dx[r[d[gx][gy]]];
		jj = jj - dy[r[d[gx][gy]]];
	}

	cv::Mat result1(mapImage.cols, mapImage.rows, CV_MAKETYPE(CV_8U, 1));
	cv::Mat result2(mapImage.cols, mapImage.rows, CV_MAKETYPE(CV_8U, 1));
	cv::Mat over(mapImage.cols, mapImage.rows, CV_MAKETYPE(CV_8U, 3));
	for(int x=0; x<mapImage.cols; x++){
		for(int y=0; y<mapImage.rows; y++){
			over.data[ y*over.step + x*over.elemSize()] = mapImage.data[ y*mapImage.step + x*mapImage.elemSize()];
			over.data[ y*over.step + x*over.elemSize()+1] = mapImage.data[ y*mapImage.step + x*mapImage.elemSize()];
			over.data[ y*over.step + x*over.elemSize()+2] = mapImage.data[ y*mapImage.step + x*mapImage.elemSize()];
		}
	}
	for(int x=0; x<mapImage.cols; x++){
		for(int y=0; y<mapImage.rows; y++){
			if(bd[x][y]!=0){
				result1.data[ y*result1.step + x*result1.elemSize()] = bd[x][y]+127;
				over.data[ y*over.step + x*over.elemSize()] = 0;
				over.data[ y*over.step + x*over.elemSize()+1] = 0;
				over.data[ y*over.step + x*over.elemSize()+2] = 255;
			}
			else result1.data[ y*result1.step + x*result1.elemSize()] = 0;
			if(bc[x][y]!=0)result2.data[ y*result2.step + x*result2.elemSize()] = bc[x][y]+127;
			else result2.data[ y*result2.step + x*result2.elemSize()] = 0;
		}
	}
	std::ofstream ofs(savePath+"bd.txt");
	for(int y=0; y<mapImage.rows; y++){
		for(int x=0; x<mapImage.cols-1; x++){
			if(bd[x][y]<10)ofs << std::to_string(bd[x][y]) << "  ";
			else ofs << std::to_string(bd[x][y]) << " ";
		}
		ofs << std::to_string(bd[mapImage.cols][y]) << std::endl;
	}
    ofs.close();
	cv::imwrite(savePath+"result1.png", result1);
	cv::imwrite(savePath+"result2.png", result2);
	cv::imwrite(savePath+"override.png", over);

	while(ros::ok()){
		ros::spinOnce();
	}


	return 0;
}
