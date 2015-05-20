#ifndef __TEXTURE_H__
#define __TEXTURE_H__

#include <fstream>
#include <iostream>
#include <sstream>
#include <stdlib.h>

#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/surface/texture_mapping.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/conversions.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "Kinect.h"

class Texture
{
	public:
		static void showMesh (pcl::PolygonMesh mesh);

		static int saveOBJFile (const std::string &file_name, const pcl::TextureMesh &tex_mesh, unsigned precision);

		static void showCameras (pcl::texture_mapping::CameraVector cams, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);

    	static void readCamPoseFile(pcl::TextureMapping<pcl::PointXYZ>::Camera &cam);
		
		static void applyCameraPose(Kinect* kinect);

		static void applyTexture(pcl::PolygonMesh &triangles, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    
    	static pcl::texture_mapping::CameraVector mCameras;
};
#endif