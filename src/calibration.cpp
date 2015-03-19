#include <iostream>
#include "libfreenect/libfreenect_sync.h"

#include <stdio.h>
#include <stdlib.h>

#include <iostream>
#include <sstream>
#include <time.h>
#include <stdio.h>

#include <unistd.h>

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
Mat nextImage(int n)
{
	Mat result_rgb;
	Mat result_ir;
	int ret;
	char *rgb = 0;
	int rows = 212;
	int columns = 162;

	uint32_t ts;

	FILE *fp;
	
	if(n == 1)
	{ 
		ret = freenect_sync_get_video((void**)&rgb, &ts, 0, FREENECT_VIDEO_RGB);
		fp = open_dump("cali_rgb.ppm");
		dump_rgb(fp, rgb, 640, 480);
		fclose(fp);
		result_rgb = imread("cali_rgb.ppm");
		resize(result_rgb, result_rgb, Size(212,162));
		return result_rgb;
	}

	else
	{ 
		ret = freenect_sync_get_video((void**)&rgb, &ts, 0, FREENECT_VIDEO_IR_8BIT);
		fp = open_dump("cali_ir.ppm");
		dump_ir(fp, rgb, 640, 480);
		fclose(fp);
		result_ir = imread("cali_ir.ppm");
		result_ir = result_ir(Range(0,162), Range(0,212));

		return result_ir;
	}
}

public:
	bool goodInput;
};

static void read(const FileNode& node, Settings& x, const Settings& default_value = Settings())
{
	
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
	Size boardSize;
	boardSize.width = 7;
	boardSize.height = 5;
	//vector<vector<Point2f>> imagePoints;
	int q;
	int counter = 0;
	int rgb_img = 1;
	int ir_img = 2;
	Mat view_rgb;
	Mat view_ir;
	//take ten images and then do the calibration. The loop should continue when the key ENTER have been pressed
	do
	{
		//RGB
		Mat view_rgb; //InputArray image
		Mat view_ir; //InputArray image
		//bool blinkOutput = false;

		view_rgb = s.nextImage(rgb_img);
		view_ir = s.nextImage(ir_img);

		//cout << "size: " << view.size() << endl;
		//imageSize = view1.size();

		vector<Point2f> pointBuf_RGB;
		vector<Point2f> pointBuf_IR;
		//findChessboardCorners(InputArray image, Size patternSize, OutputArray corners, int flags=CALIB_CB_ADAPTIVE_THRESH+CALIB_CB_NORMALIZE_IMAGE )
		bool found_rgb = findChessboardCorners(view_rgb, boardSize, pointBuf_RGB);
		bool found_ir = findChessboardCorners(view_ir, boardSize, pointBuf_IR);

		if(found_rgb && found_ir)
		{
			counter++;
			cout << "hej" << endl;
			
			//view_rgb *= 1./255;
			//Mat viewGray;
			//cvtColor(view, viewGray, COLOR_BGR2GRAY);

 			//cornerSubPix(view, pointBuf, Size(11, 11), Size(-1, -1),
    		//TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
			
			drawChessboardCorners(view_rgb, boardSize, Mat(pointBuf_RGB), found_rgb);
			//view = s.nextImage(v);
			//the user needs to press a or q to continue the loop. q = break the loop
/*			q = keyPressed();
			if(q == -1) break;*/
			#ifdef _APPLE_ 
				namedWindow( "RGB", WINDOW_AUTOSIZE );
				imshow("RGB", view_rgb);

				drawChessboardCorners(view_ir, boardSize, Mat(pointBuf_IR), found_ir);
				namedWindow( "IR", WINDOW_AUTOSIZE );
				imshow("IR", view_ir);
				waitKey(0);
			#else
				drawChessboardCorners(view_ir, boardSize, Mat(pointBuf_IR), found_ir);
				//view = s.nextImage(v);
				//the user needs to press a or q to continue the loop. q = break the loop
				//q = keyPressed();
				//if(q == -1) break;
				imwrite("rgb_corners.ppm", view_rgb);
				imwrite("ir_corners.ppm", view_ir);

			#endif
		}

	}while(counter != 1);



	cout << "Calibration done!" << endl;

	return 0;
}