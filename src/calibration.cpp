#include <iostream>
#include "libfreenect/libfreenect_sync.h"

#include <stdio.h>
#include <stdlib.h>

#include <iostream>
#include <sstream>
#include <time.h>
#include <stdio.h>

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

void dump_depth(FILE *fp, void *data, unsigned int width, unsigned int height)
{
	fprintf(fp, "P5 %u %u 65535\n", width, height);
	fwrite(data, width * height * 2, 1, fp);
}

class Settings
{
public: 
	Settings () : goodInput(false) {}
	enum Pattern { NOT_EXISTING, CHESSBOARD};
	enum InputType {INVALID, CAMERA, VIDEO_FILE, IMAGE_LIST};

	void write(FileStorage& fs) const
	{
		
	}

	void read(const FileNode& node)
	{

	}

	//take the next image from the Kinect
Mat nextImage()
{
	Mat result;
	int ret;
	char *rgb = 0;

	uint32_t ts;

	FILE *fp;	
	
	ret = freenect_sync_get_video((void**)&rgb, &ts, 0, FREENECT_VIDEO_RGB);

	fp = open_dump("cali.ppm");
	dump_rgb(fp, rgb, 640, 480);
	fclose(fp);

	result = imread("cali.ppm");

	return result;
}

public:
	bool goodInput;
};

static void read(const FileNode& node, Settings& x, const Settings& default_value = Settings())
{
	
}

int keyPressed()
{
	char s;

	do
	{
		cout << "Press a to take next image or press q to quit" << endl;
		cin >> s;
	}while((s != 'a') && (s != 'q'));

	if(s == 'q')
	{
		return -1;
	}else return 1;
}

int main(int argc, char const *argv[])
{
	//text in the beginning about the file
	help();
	Settings s;
	//open default.yml as default
	const string inputFile = argc > 1 ? argv[1] : "default.yml";
	//read the settings
	FileStorage fs(inputFile, FileStorage::READ);

	if(!fs.isOpened())
	{
		cout << "Could not open the file, sorry!" << endl;
		return -1;
	}
	fs["Settings"] >> s;
	fs.release();

	//if(!s.goodInput)
	//{
	//	cout << "Invalid input detected. Application stopping" << endl;
	//	return -1;
	//}

	freenect_sync_set_tilt_degs(0, 0);
	freenect_raw_tilt_state *state = 0;	

	Size imageSize;
	//vector<vector<Point2f>> imagePoints;
	int q;
	//take ten images and then do the calibration. The loop should continue when the key ENTER have been pressed
	for(int i = 0; ; ++i)
	{
		Mat view; //InputArray image
		bool blinkOutput = false;

		view = s.nextImage();
		//cout << "size: " << view.size() << endl;
		imageSize = view.size();

		vector<Point2f> pointBuf;
		//findChessboardCorners(InputArray image, Size patternSize, OutputArray corners, int flags=CALIB_CB_ADAPTIVE_THRESH+CALIB_CB_NORMALIZE_IMAGE )
		bool found = findChessboardCorners(view, imageSize, pointBuf, 
			CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FAST_CHECK | CALIB_CB_NORMALIZE_IMAGE);

		if(found)
		{
			cout << "hej" << endl;
		}

		//the user needs to press a or q to continue the loop. q = break the loop
		//q = keyPressed();
		//if(q == -1) break;

	}

	cout << "Calibration done!" << endl;

	return 0;
}