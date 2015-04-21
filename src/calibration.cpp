#include <iostream>
#include "libfreenect/libfreenect_sync.h"

#include <stdio.h>
#include <stdlib.h>

#include <iostream>
#include <sstream>
#include <time.h>
#include <stdio.h>

#include <cmath>

#include <unistd.h>

#include <string>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

#ifndef _CRT_SECURE_NO_WARNINGS
# define _CRT_SECURE_NO_WARNINGS
#endif

//need one rgb-image and one depth-image

using namespace std;
using namespace cv;

const int HEIGHT = 480;
const int WIDTH = 640;

FILE *open_dump(const char *filename)
{
	//open file
	FILE* fp = fopen(filename, "w");
	if (fp == NULL) {
		fprintf(stderr, "Error: Cannot open file [%s]\n", filename);
		exit(1);
	}
	printf("%s\n", filename);
	return fp;
}

void dump_rgb(FILE *fp, void *data, unsigned int width, unsigned int height)
{
	//*3 = channel
	fprintf(fp, "P6 %u %u 255\n", width, height);
	//write to file
	fwrite(data, width * height * 3, 1, fp);
}

Mat nextImage_RGB()
{
	Mat result_rgb;
	int ret;
	char *rgb = 0;

	uint32_t ts;

	FILE *fp;

	ret = freenect_sync_get_video((void**)&rgb, &ts, 0, FREENECT_VIDEO_RGB);
	fp = open_dump("cali_rgb.jpg");
	dump_rgb(fp, rgb, WIDTH, HEIGHT);
	fclose(fp);
	result_rgb = imread("cali_rgb.jpg");
	resize(result_rgb, result_rgb, Size(WIDTH/3,HEIGHT/3));

	return result_rgb;
}

//create a matrix, 35x3
Mat Matrix(vector<Point2f> v)
{
	Mat tmp(35,3, CV_64F);
	for(int i = 0; i < 35; i++)
	{
		tmp.at<double>(i,0) = v.at(i).x;
		tmp.at<double>(i,1) = v.at(i).y;
		tmp.at<double>(i,2) = 1;
	}
	return tmp;
}
//converting of MATLAB-code for calibration
//change name after
vector<Point2f> MATLAB(vector<Point2f> RGB, vector<Point2f> IR)
{
	vector<Point2f> temp;

	Mat rgb = Matrix(RGB);
	Mat ir = Matrix(IR);
	Mat x = rgb.inv() * ir;

	return temp;
}

int main(int argc, char const *argv[])
{

	Size boardSize(7,5); //how many corners that have to be found
	int counter = 0;
	int rgb_img = 1; //for function nextImage

	do
	{
		Mat view_rgb; //InputArray image

		view_rgb = nextImage_RGB(); //

		vector<Point2f> pointBuf_RGB;

		bool found_rgb = findChessboardCorners(view_rgb, boardSize, pointBuf_RGB);

		if(found_rgb)
		{
			counter++; //counter for every image that have been taken
			cout << "Bild " << counter << " tagen" << endl;
			drawChessboardCorners(view_rgb, boardSize, Mat(pointBuf_RGB), found_rgb);

			cout << pointBuf_RGB << endl;
			string RGB_name = "rgb_corners.ppm";

			//save image
			imwrite(RGB_name, view_rgb);

			cout << "skriver!" << endl;
		}
	}while(counter != 10); //stop loop when 10 images have been taken

	cout << "Calibration done!" << endl;

	return 0;
}