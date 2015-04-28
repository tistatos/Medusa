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

Mat nextImage_RGB(int counter)
{
	Mat result_rgb;
	// char *rgb = 0;
	// uint32_t ts;
	// FILE *fp;

	// freenect_sync_get_video((void**)&rgb, &ts, 0, FREENECT_VIDEO_RGB);

	char filename[64];
	sprintf(filename, "cali_%i.ppm", counter);
	// fp = open_dump(filename);
	// dump_rgb(fp, rgb, WIDTH, HEIGHT);
	// fclose(fp);
	result_rgb = imread(filename);

	return result_rgb;
}

int main(int argc, char const *argv[])
{

	Size boardSize(7,5); //how many corners that have to be found
	Size imageSize(640,480);
	int counter = 0;

	time_t start = time(0);
	vector<vector<Point2f> > _imagePoints;

	Mat view_rgb; //InputArray image

	cout << "Calibration started" << endl;

	do
	{
		view_rgb = nextImage_RGB(counter); //

		vector<Point2f> pointBuf_RGB;

		bool found_rgb = findChessboardCorners(view_rgb, boardSize, pointBuf_RGB);

		double seconds_since_start = difftime( time(0), start);
		if(found_rgb && seconds_since_start > 1.0)
		{
			counter++; //counter for every image that have been taken
			cout << "Bild " << counter << " tagen" << endl;
			drawChessboardCorners(view_rgb, boardSize, Mat(pointBuf_RGB), found_rgb);

			_imagePoints.push_back(pointBuf_RGB);

			// char filename[64];
			// sprintf(filename, "%i_chess.ppm", counter);
			// string RGB_name = filename;
			// //save image
			// imwrite(RGB_name, view_rgb);
			// start = time(0);
			// cout << "skriver!" << endl;
		}
	}while(counter != 10); //stop loop when 10 images have been taken

	cout << "Calibration done!" << endl;

	Mat cameraMatrix;
	Mat distCoeffs;
	vector< Mat > rvecs, tvecs;
 	cameraMatrix = Mat::eye(3, 3, CV_64F);
 	distCoeffs = Mat::zeros(8, 1, CV_64F);


	vector<Point3f> corners;
  corners.resize(0);
	float squareSize = 20.0f;
  for( int i = 0; i < boardSize.height; ++i )
    for( int j = 0; j < boardSize.width; ++j )
        corners.push_back(Point3f(float( j*squareSize ), float( i*squareSize ), 0));

  vector<vector<Point3f> > objectPoints(0);
  objectPoints.resize(_imagePoints.size(),corners);

	double rms = calibrateCamera(objectPoints, _imagePoints, imageSize, cameraMatrix,
                            distCoeffs, rvecs, tvecs);

	cout << rms << endl;
	cout << "Camera matrix:" << endl <<  cameraMatrix << endl;
	cout << "Coefficents:" << endl <<  distCoeffs << endl;

	Mat rvecs2, tvecs2;
	solvePnP(objectPoints.at(4), _imagePoints.at(4), cameraMatrix, distCoeffs, rvecs2, tvecs2);


	Mat rotation, viewMatrix(4, 4, CV_64F);
	Rodrigues(rvecs2, rotation);

	for(unsigned int row=0; row<3; ++row)
	{
	   for(unsigned int col=0; col<3; ++col)
	   {
	      viewMatrix.at<double>(row, col) = rotation.at<double>(row, col);
	   }
	   viewMatrix.at<double>(row, 3) = tvecs2.at<double>(row, 0);
	}
	viewMatrix.at<double>(3, 3) = 1.0f;
	viewMatrix.at<double>(3, 0) = 0.0f;
	viewMatrix.at<double>(3, 1) = 0.0f;
	viewMatrix.at<double>(3, 2) = 0.0f;



	cout << viewMatrix << endl;

	vector<Point2f> reproj_points;

	view_rgb = nextImage_RGB(4); //

	projectPoints(objectPoints.at(4), rvecs2, tvecs2, cameraMatrix, distCoeffs, reproj_points);
	drawChessboardCorners(view_rgb, boardSize, Mat(reproj_points), true);


		imwrite("reproj.ppm", view_rgb);

	return 0;
}