#include <iostream>
#include "libfreenect/libfreenect_sync.h"

#include <stdio.h>
#include <stdlib.h>

#include <iostream>
#include <sstream>
#include <string>
#include <time.h>
#include <stdio.h>

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
		fp = open_dump("cali_rgb.ppm");
		dump_rgb(fp, rgb, width, height);
		fclose(fp);
		result_rgb = imread("cali_rgb.ppm");
		resize(result_rgb, result_rgb, Size(width/3,height/3));
		return result_rgb;
	}

	else
	{ 
		ret = freenect_sync_get_video((void**)&rgb, &ts, 0, FREENECT_VIDEO_IR_8BIT);
		fp = open_dump("cali_ir.ppm");
		dump_ir(fp, rgb, width, height);
		fclose(fp);
		result_ir = imread("cali_ir.ppm");
		result_ir = result_ir(Range(0,height/3), Range(0,width/3));

		return result_ir;
	}
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

string convertInt(int number)
{
   stringstream ss;
   ss << number;
   return ss.str();
}

int main(int argc, char const *argv[])
{
	//text in the beginning about the file
	help();

	freenect_sync_set_tilt_degs(0, 0);
	freenect_raw_tilt_state *state = 0;	

	Size boardSize(7,5); //how many corners that have to be found
	int counter = 0;
	int rgb_img = 1; //for function nextImage
	int ir_img = 2; //for function nextImage
	do
	{
		Mat view_rgb; //InputArray image
		Mat view_ir; //InputArray image
		char bajs;

		view_rgb = nextImage(rgb_img); //
		view_ir = nextImage(ir_img);

		vector<Point2f> pointBuf_RGB;
		vector<Point2f> pointBuf_IR;
		
		bool found_rgb = findChessboardCorners(view_rgb, boardSize, pointBuf_RGB);
		bool found_ir = findChessboardCorners(view_ir, boardSize, pointBuf_IR);

		if(found_rgb && found_ir)
		{
			counter++; //counter for every image that have been taken
			cout << "Bild " << counter << " tagen" << endl;
			cout << endl << pointBuf_RGB << endl;
			cout << endl << pointBuf_IR << endl;
			imwrite("testrgb.jpg", view_rgb);
			imwrite("testir.jpg", view_ir);
			cin >> bajs;

			//find chessboardcorners and draw lines in view_rgb and view_ir
			drawChessboardCorners(view_rgb, boardSize, Mat(pointBuf_RGB), found_rgb);
			drawChessboardCorners(view_ir, boardSize, Mat(pointBuf_IR), found_ir);

			//make a string for the filename
			string RGB_name = "rgb_corners" + convertInt(counter) + ".ppm";
			string IR_name = "ir_corners" + convertInt(counter) + ".ppm";
			//save image
			imwrite(RGB_name, view_rgb);
			imwrite(IR_name, view_ir);
		}
	}while(counter != 10); //stop loop when 10 images have been taken 

	cout << "Calibration done!" << endl;

	return 0;
}