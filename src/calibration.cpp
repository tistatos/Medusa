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
	cv::Mat view_rgb = fromPNGtoMat(frame);
	std::vector<cv::Point2f> pointBuf_RGB;

	bool found_rgb = findChessboardCorners(view_rgb, mBoardSize, pointBuf_RGB);
	if(found_rgb)
	{
		mImagePoints.push_back(pointBuf_RGB);
		mProcessedImages++; //counter for every image that have been taken
		std::cout << "Found chess: " << mProcessedImages << std::endl;
	}

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

		double rms = calibrateCamera(getObjectPoints(), mImagePoints, mImageSize, mCameraMatrix,
                            			mDistCoeffs, rvecs, tvecs);
		mCalibrated = true;
		std::cout << "Cam Matrix: " << mCameraMatrix << std::endl;

		std::cout << "Cam dist: " << mDistCoeffs << std::endl;
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
cv::Mat Calibration::getCameraExtrinsic(VIDEO_IMAGE frame)
{
	if(!mCalibrated)
		return cv::Mat::zeros(3, 3, CV_64F);

	//cv::Mat image = fromPNGtoMat(frame);
	int processedImages = mProcessedImages;
	processImage(frame);
	if(processedImages == mProcessedImages)
		return cv::Mat::zeros(3, 3, CV_64F);

	cv::Mat rvec, tvec;
	cv::solvePnP(getObjectPoints().at(0), mImagePoints.at(mImagePoints.size()-1), mCameraMatrix, mDistCoeffs, rvec, tvec);

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

cv::Mat Calibration::fromPNGtoMat(VIDEO_IMAGE frame)
{
	cv::Mat imageMatrix = cv::Mat::zeros(640,480, CV_8UC3);
	uint8_t* pixelPtr = (uint8_t*)imageMatrix.data;
	for (int i = 0; i < 480; ++i)
	{
		for (int j = 0; j < 640; ++j)
		{
			pixelPtr[i*imageMatrix.cols*3 + j*3 + 0] = frame[i][j].blue;//B
			pixelPtr[i*imageMatrix.cols*3 + j*3 + 1] = frame[i][j].green;//G
			pixelPtr[i*imageMatrix.cols*3 + j*3 + 2] = frame[i][j].red;//R
		}
	}
	return imageMatrix;
}