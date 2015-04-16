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

void help()
{
	cout << "Trying to make a calibration!" << endl;
}

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

void dump_ir(FILE *fp, void *data, unsigned int width, unsigned int height)
{
	fprintf(fp, "P6 %u %u 255\n", width, height);
	//write to file
	fwrite(data, width * height, 1, fp);
}

void dump_depth(FILE *fp, void *data, unsigned int width, unsigned int height)
{
	fprintf(fp, "P5 %u %u 65535\n", width, height);
	fwrite(data, width * height * 2, 1, fp);
}

Mat nextImage(int n)
{
	Mat result_rgb;
	Mat result_ir;
	int ret;
	char *rgb = 0;
	int width = 640;
	int height = 480;

	uint32_t ts;

	FILE *fp;

	if(n == 1)
	{
		ret = freenect_sync_get_video((void**)&rgb, &ts, 0, FREENECT_VIDEO_RGB);
		fp = open_dump("cali_rgb.jpg");
		dump_rgb(fp, rgb, WIDTH, HEIGHT);
		fclose(fp);
		result_rgb = imread("cali_rgb.jpg");
		resize(result_rgb, result_rgb, Size(WIDTH/3,HEIGHT/3));
		return result_rgb;
	}

	else if(n == 2)
	{
		ret = freenect_sync_get_video((void**)&rgb, &ts, 0, FREENECT_VIDEO_IR_8BIT);
		fp = open_dump("cali_ir.jpg");
		dump_ir(fp, rgb, WIDTH, HEIGHT);
		fclose(fp);
		result_ir = imread("cali_ir.jpg");
		result_ir = result_ir(Range(0,HEIGHT/3), Range(0,WIDTH/3));

		return result_ir;
	}
	else if(n == 3)
	{
		ret = freenect_sync_get_video((void**)&rgb, &ts, 0, FREENECT_VIDEO_IR_8BIT);
		fp = open_dump("cali_ir.jpg");
		dump_ir(fp, rgb, WIDTH, HEIGHT);
		fclose(fp);
		result_ir = imread("cali_ir.jpg");
		result_ir = result_ir(Range(0,HEIGHT/3), Range(WIDTH/3,2*(WIDTH/3)));

		return result_ir;

	}
	else
	{
		ret = freenect_sync_get_video((void**)&rgb, &ts, 0, FREENECT_VIDEO_IR_8BIT);
		fp = open_dump("cali_ir.jpg");
		dump_ir(fp, rgb, WIDTH, HEIGHT);
		fclose(fp);
		result_ir = imread("cali_ir.jpg");
		result_ir = result_ir(Range(0,HEIGHT/3), Range(2*(WIDTH/3),WIDTH));

		return result_ir;
	}
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

Mat nextImage_IR()
{
	Mat result_ir;
	int ret;
	char *ir = 0;

	uint32_t ts;

	FILE *fp;
	ret = freenect_sync_get_video((void**)&ir, &ts, 0, FREENECT_VIDEO_IR_8BIT);
	fp = open_dump("cali_ir.jpg");
	dump_ir(fp, ir, WIDTH, HEIGHT);
	fclose(fp);
	result_ir = imread("cali_ir.jpg");
	result_ir = result_ir(Range(0,HEIGHT/3), Range(0,WIDTH/3));

	return result_ir;
}

int keyPressed()
{
	char key;

	do
	{
		cout << "Press a to take next image or press q to quit" << endl;
		cin >> key;
	}while((key != 'a') && (key != 'q'));

	if(key == 'q')
	{
		return -1;
	}else return 1;
}

vector<Point2f> calibration_ir_one_image(vector<Point2f> ir1, vector<Point2f> ir2, vector<Point2f> ir3)
{
	vector<Point2f> temp;
	for(int i = 0; i < ir1.size(); i++)
	{
		temp.push_back((ir1.at(i) + ir2.at(i) + ir3.at(i))* (1/ 3) );
	}
	//cout << ir1.at(1).x << " " << ir2.at(1).x << " " << ir3.at(1).x << endl;
	return temp;
}

string intToString(int number)
{
   stringstream ss;
   ss << number;
   return ss.str();
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
	//text in the beginning about the file
	help();

	//freenect_sync_set_tilt_degs(0, 0);
	//freenect_raw_tilt_state *state = 0;

	Size boardSize(7,5); //how many corners that have to be found
	int counter = 0;
	int rgb_img = 1; //for function nextImage
	int ir_img1 = 2; //for function nextImage
	int ir_img2 = 3;
	int ir_img3 = 4;
	do
	{
		Mat view_rgb; //InputArray image
		Mat view_ir1; //InputArray image
		Mat view_ir2;
		Mat view_ir3;

		//TODO: Gör två funktioner för rgb resp. ir
		view_rgb = nextImage(rgb_img); //
		view_ir1 = nextImage(ir_img1);
		view_ir2 = nextImage(ir_img2);
		view_ir3 = nextImage(ir_img3);

		vector<Point2f> pointBuf_RGB;
		vector<Point2f> pointBuf_IR1;
		vector<Point2f> pointBuf_IR2;
		vector<Point2f> pointBuf_IR3;
		vector<Point2f> pointBuf_IR;

		vector<Point2f> point4;

		bool found_rgb = findChessboardCorners(view_rgb, boardSize, pointBuf_RGB);
		bool found_ir1 = findChessboardCorners(view_ir1, boardSize, pointBuf_IR1);
		bool found_ir2 = findChessboardCorners(view_ir2, boardSize, pointBuf_IR2);
		bool found_ir3 = findChessboardCorners(view_ir3, boardSize, pointBuf_IR3);

		if(found_rgb && found_ir1 && found_ir2 && found_ir3)
		{
			counter++; //counter for every image that have been taken
			cout << "Bild " << counter << " tagen" << endl;
			//cout << endl << pointBuf_RGB << endl;
			//cout << endl << pointBuf_IR << endl;
			//imwrite("testrgb.jpg", view_rgb);
			//imwrite("testir.jpg", view_ir);
			pointBuf_IR = calibration_ir_one_image(pointBuf_IR1, pointBuf_IR2, pointBuf_IR3);
			point4 = MATLAB(pointBuf_RGB, pointBuf_IR);
			//find chessboardcorners and draw lines in view_rgb and view_ir
			drawChessboardCorners(view_rgb, boardSize, Mat(point4), found_rgb);
			//drawChessboardCorners(view_ir1, boardSize, Mat(pointBuf_IR1), found_ir1);
			//drawChessboardCorners(view_ir2, boardSize, Mat(pointBuf_IR2), found_ir2);
			drawChessboardCorners(view_ir3, boardSize, Mat(pointBuf_RGB), found_ir3);

			//make a string for the filename
			string RGB_name = "rgb_corners" + intToString(counter) + ".jpg";
			//string IR_name1 = "ir_corners1" + intToString(counter) + ".jpg";
			//string IR_name2 = "ir_corners2" + intToString(counter) + ".jpg";
			string IR_name3 = "ir_corners3" + intToString(counter) + ".jpg";

			//save image
			imwrite(RGB_name, view_rgb);
			//imwrite(IR_name1, view_ir1);
			//imwrite(IR_name2, view_ir2);
			imwrite(IR_name3, view_ir3);
		}
	}while(counter != 1); //stop loop when 10 images have been taken

	cout << "Calibration done!" << endl;

	return 0;
}