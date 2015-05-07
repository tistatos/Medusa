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

#include "calibration.h"

/**
 * @brief Constructor of calibrator
 *
 * @param nPictures number of pictures used for calibration
 * @param width width of images
 * @param height height of images
 * @param boardX grid size of chessboard
 * @param boardY grid size
 */
Calibration::Calibration(int nPictures, int width, int height, int boardX, int boardY):
mImageSize(width,height), mBoardSize(boardX,boardY), mNumberOfPictures(nPictures)
{
	mGridSize = 29.0f;
	clear();
}

/**
 * @brief reset calibration parameters
 */
void Calibration::clear()
{
	mCameraMatrix = cv::Mat::eye(3, 3, CV_64F);
 	mDistCoeffs = cv::Mat::zeros(8, 1, CV_64F);
 	mProcessedImages = 0;
 	mCalibrated = false;
}

/**
 * @brief process image sent to calibrator
 * @todo currently save and loads a temp.png fix this
 *
 * @param frame frame to process
 * @return number of processed images
 */
int Calibration::processImage(VIDEO_IMAGE frame)
{
	//FIXME will cause errors with multiple kinects perhaps
	frame.write("temp.png");
	cv::Mat view_rgb;
	view_rgb= cv::imread("temp.png");

	std::vector<cv::Point2f> pointBuf_RGB;

	bool found_rgb = findChessboardCorners(view_rgb, mBoardSize, pointBuf_RGB);
	if(found_rgb)
	{
		//drawChessboardCorners(view_rgb, mBoardSize, cv::Mat(pointBuf_RGB), found_rgb);
		mImagePoints.push_back(pointBuf_RGB);
		mProcessedImages++; //counter for every image that have been taken
		std::cout << "Found chess: " << mProcessedImages << std::endl;
	}

	std::cout << "processed" << std::endl;

	return mProcessedImages;
}

/**
 * @brief use the processed images to calibrate camera
 *
 * @return residual of calibration, -1 if incorrect number of images has been processed
 */
double Calibration::calibrate()
{

	if(mProcessedImages >= mNumberOfPictures)
	{
		std::vector<cv::Mat > rvecs, tvecs;

		std::cout << "number of imagePoints: " << mImagePoints.size() << std::endl;
		std::cout << "number of imagePoint corners: " << mImagePoints[0].size() << std::endl;

		double rms = calibrateCamera(getObjectPoints(), mImagePoints, mImageSize, mCameraMatrix,
                            			mDistCoeffs, rvecs, tvecs);
		mCalibrated = true;
		return rms;
	}
	else
	{
		std::cout << "cant Calibrate" << std::endl;
		return -1;
	}
}

/**
 * @brief object points for the chessboar
 * @return returns array of arrays size determined by chess size and number of images
 */
std::vector<std::vector<cv::Point3f> > Calibration::getObjectPoints()
{
	std::vector<cv::Point3f> corners;

  corners.resize(0);

  for( int i = 0; i < mBoardSize.height; ++i )
    for( int j = 0; j < mBoardSize.width; ++j )
        corners.push_back(cv::Point3f(float( j*mGridSize ), float( i*mGridSize ), 0));

  std::vector<std::vector<cv::Point3f> > objectPoints(0);
  objectPoints.resize(mImagePoints.size(),corners);
  return objectPoints;
}

/**
 * @brief Calculate camera extrinsics
 * @todo  should use all images to calculate extrinsics
 * @details [long description]
 * @return matrix with extrinsic data
 */
cv::Mat Calibration::getCameraExtrinsic()
{

	if(!mCalibrated)
		return cv::Mat::eye(3, 3, CV_64F);

	cv::Mat rvec, tvec;
	cv::solvePnP(getObjectPoints().at(0), mImagePoints.at(0), mCameraMatrix, mDistCoeffs, rvec, tvec);

	cv::Mat rotation, viewMatrix(4, 4, CV_64F);
	cv::Rodrigues(rvec, rotation);

	for(unsigned int row=0; row<3; ++row)
	{
	   for(unsigned int col=0; col<3; ++col)
	   {
	      viewMatrix.at<double>(row, col) = rotation.at<double>(row, col);
	   }
	   viewMatrix.at<double>(row, 3) = tvec.at<double>(row, 0);
	}
	viewMatrix.at<double>(3, 3) = 1.0f;
	viewMatrix.at<double>(3, 0) = 0.0f;
	viewMatrix.at<double>(3, 1) = 0.0f;
	viewMatrix.at<double>(3, 2) = 0.0f;

	return  viewMatrix;
}


////////////////
/// OLD CODE
////////////////


// using namespace std;
// using namespace cv;

// const int HEIGHT = 480;
// const int WIDTH = 640;





// Mat nextImage_RGB(int counter)
// {
// 	Mat result_rgb;
// 	// char *rgb = 0;
// 	// uint32_t ts;
// 	// FILE *fp;

// 	// freenect_sync_get_video((void**)&rgb, &ts, 0, FREENECT_VIDEO_RGB);

// 	char filename[64];
// 	sprintf(filename, "cali_%i.ppm", counter);
// 	// fp = open_dump(filename);
// 	// dump_rgb(fp, rgb, WIDTH, HEIGHT);
// 	// fclose(fp);
// 	result_rgb = imread(filename);

// 	return result_rgb;
// }

// int main(int argc, char const *argv[])
// {

// 	Size boardSize(7,5); //how many corners that have to be found
// 	Size imageSize(640,480);
// 	int counter = 0;

// 	time_t start = time(0);
// 	vector<vector<Point2f> > _imagePoints;

// 	Mat view_rgb; //InputArray image

// 	cout << "Calibration started" << endl;

// 	do
// 	{
// 		view_rgb = nextImage_RGB(counter); //

// 		vector<Point2f> pointBuf_RGB;

// 		bool found_rgb = findChessboardCorners(view_rgb, boardSize, pointBuf_RGB);

// 		double seconds_since_start = difftime( time(0), start);
// 		if(found_rgb && seconds_since_start > 1.0)
// 		{
// 			counter++; //counter for every image that have been taken
// 			cout << "Bild " << counter << " tagen" << endl;
// 			drawChessboardCorners(view_rgb, boardSize, Mat(pointBuf_RGB), found_rgb);

// 			_imagePoints.push_back(pointBuf_RGB);

// 			// char filename[64];
// 			// sprintf(filename, "%i_chess.ppm", counter);
// 			// string RGB_name = filename;
// 			// //save image
// 			// imwrite(RGB_name, view_rgb);
// 			// start = time(0);
// 			// cout << "skriver!" << endl;
// 		}
// 	}while(counter != 10); //stop loop when 10 images have been taken

// 	cout << "Calibration done!" << endl;

// 	Mat cameraMatrix;
// 	Mat distCoeffs;
// 	vector< Mat > rvecs, tvecs;
//  	cameraMatrix = Mat::eye(3, 3, CV_64F);
//  	distCoeffs = Mat::zeros(8, 1, CV_64F);


// 	vector<Point3f> corners;
//   corners.resize(0);
// 	float squareSize = 20.0f;
//   for( int i = 0; i < boardSize.height; ++i )
//     for( int j = 0; j < boardSize.width; ++j )
//         corners.push_back(Point3f(float( j*squareSize ), float( i*squareSize ), 0));

//   vector<vector<Point3f> > objectPoints(0);
//   objectPoints.resize(_imagePoints.size(),corners);

// 	double rms = calibrateCamera(objectPoints, _imagePoints, imageSize, cameraMatrix,
//                             distCoeffs, rvecs, tvecs);

// 	cout << rms << endl;
// 	cout << "Camera matrix:" << endl <<  cameraMatrix << endl;
// 	cout << "Coefficents:" << endl <<  distCoeffs << endl;

// 	Mat rvecs2, tvecs2;
// 	solvePnP(objectPoints.at(4), _imagePoints.at(4), cameraMatrix, distCoeffs, rvecs2, tvecs2);


// 	Mat rotation, viewMatrix(4, 4, CV_64F);
// 	Rodrigues(rvecs2, rotation);

// 	for(unsigned int row=0; row<3; ++row)
// 	{
// 	   for(unsigned int col=0; col<3; ++col)
// 	   {
// 	      viewMatrix.at<double>(row, col) = rotation.at<double>(row, col);
// 	   }
// 	   viewMatrix.at<double>(row, 3) = tvecs2.at<double>(row, 0);
// 	}
// 	viewMatrix.at<double>(3, 3) = 1.0f;
// 	viewMatrix.at<double>(3, 0) = 0.0f;
// 	viewMatrix.at<double>(3, 1) = 0.0f;
// 	viewMatrix.at<double>(3, 2) = 0.0f;



// 	cout << viewMatrix << endl;

// 	vector<Point2f> reproj_points;

// 	view_rgb = nextImage_RGB(4); //

// 	projectPoints(objectPoints.at(4), rvecs2, tvecs2, cameraMatrix, distCoeffs, reproj_points);
// 	drawChessboardCorners(view_rgb, boardSize, Mat(reproj_points), true);


// 		imwrite("reproj.ppm", view_rgb);

// 	return 0;
// }