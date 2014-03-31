/*********************************************************************
*
*  Copyright (c) 2011, Jeannette Bohg - KTH Stockholm
*  (bohg@csc.kth.se)
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Jeannette Bohg nor the names of KTH
*     may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
#include <stdio.h>
#include <fstream>
#include <iostream>
#include <vector>
#include <string>
#include "parse.h"
#include "predictObject.h"
#include "utilities.h"
#include <libgen.h>

#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/PointCloud2.h>

//#include <terminal_tools/parse.h>
#include <pcl/io/pcd_io.h>

#define BUFSIZE 8096
#define VOXSIZE 10
#define MINPTS 90
 
std::vector<cv::Point3f> point_vec;
std::vector<cv::Point3f> normals;
std::vector<float> plausibilities;

std::string targetDir;
int         mDepth = 5;

bool _use_plausibilities = false;

// Functions for parsing the input arguments
bool parseParameters(int argc, char**argv);
bool readOptArgs(int argc, char**argv, int offset);
void errorMsg();

int main(int argc, char**argv){

    std::string newName;
    std::string origName;
  
    if(!parseParameters(argc, argv))
	exit(-1);
    else {
	std::string plaus_add;
	
	if(_use_plausibilities)
	{
	    std::cout << "Creating prediction object that uses plausibilities " << std::endl;
	    plaus_add = "_withPlaus";
	}
	else { 
	    std::cout << "Creating prediction object that does not use plausibilities " << std::endl;
	    plaus_add = "_noPlaus";

	}
	// Print Parameters
	origName = std::string(basename(argv[1]));
	int outLength = origName.size()-4;
	newName = origName.substr(0,outLength)+plaus_add;

	std::cout << "Target Directory: " << targetDir << std::endl;
	std::cout << "Output name: " << newName << std::endl;
	std::cout << "Tree Depth for approximation: " << mDepth 
		  << std::endl;

    }
  
    
    std::cout << "All Vectors should have the same size " << 
	point_vec.size() << "=" << normals.size() << "=" 
	      << plausibilities.size() << std::endl;
    
    

    PredictObject pred_obj( point_vec, 
			    normals,
			    plausibilities,
			    0, 
			    mDepth,
			    _use_plausibilities,
			    targetDir.c_str(), 
			    newName.c_str());
    
    pred_obj.ConstructMesh();
    
}

bool parseParameters(int argc, char**argv){
  
    if( argc<4 || argc>5 )
    {
	errorMsg();
	return false;
    }
    
    int idx=0;
    // Parse the command line arguments for .pcd files
    std::vector<int> p_file_indices;
    p_file_indices 
	= parse_file_extension_argument(argc,argv,".pcd");
    if (p_file_indices.size () == 0)
    {
	fprintf(stderr,"No .PCD file given.\n");
	errorMsg();
	return false;
    } else if(p_file_indices.size()>1) {
	fprintf(stderr,"More than one .PCD file given.\n");
	errorMsg();
	return false;
    }

    
    // Read point cloud data
    pcl::PCDReader pcd;
    sensor_msgs::PointCloud2 cloud;
    Eigen::Vector4f origin;
    Eigen::Quaternionf orientation;
    int version;
    std::stringstream cloud_name;

    ROS_INFO("Reading pcd file %s\n", argv[p_file_indices.at(0)]);
    if (pcd.read (argv[p_file_indices.at(0)], 
		  cloud, origin, orientation, version) < 0)
	return false;

    sensor_msgs::PointCloud2::ConstPtr point_cloud_ptr = 
	boost::make_shared<const sensor_msgs::PointCloud2> (cloud);
    
    //ROS_ASSERT(point_cloud_ptr->is_dense);
    
    int w = point_cloud_ptr->width;
    int h = point_cloud_ptr->height;
    
    ROS_INFO("Received point cloud of size %ld", 
	     (long int)w*h);
    
    point_vec.resize(w*h);
    normals.resize(w*h);
    plausibilities.resize(w*h);

    for(int y=0; y<h; ++y)
    {
	for(int x=0; x<w; ++x)
	{
	    int i = y*w+x;
	    cv::Point3f p;
	    cv::Point3f n;
	    float rgb;
	    memcpy(&p.x, &point_cloud_ptr->data[i * point_cloud_ptr->point_step + point_cloud_ptr->fields[0].offset], sizeof (float));
	    memcpy (&p.y, &point_cloud_ptr->data[i * point_cloud_ptr->point_step + point_cloud_ptr->fields[1].offset], sizeof (float));
	    memcpy (&p.z, &point_cloud_ptr->data[i * point_cloud_ptr->point_step + point_cloud_ptr->fields[2].offset], sizeof (float));
	    point_vec.at(i)=p;
	    memcpy (&rgb, &point_cloud_ptr->data[i * point_cloud_ptr->point_step + point_cloud_ptr->fields[3].offset], sizeof (float));
	    float plaus=getPlausFromRGB(rgb); 
	    plausibilities.at(i) = plaus;
	    memcpy(&n.x, &point_cloud_ptr->data[i * point_cloud_ptr->point_step + point_cloud_ptr->fields[4].offset], sizeof (float));
	    memcpy (&n.y, &point_cloud_ptr->data[i * point_cloud_ptr->point_step + point_cloud_ptr->fields[5].offset], sizeof (float));
	    memcpy (&n.z, &point_cloud_ptr->data[i * point_cloud_ptr->point_step + point_cloud_ptr->fields[6].offset], sizeof (float));
	    normals.at(i)=n;
	}
    }

    idx = p_file_indices.at(0);
    
    mDepth = atof(argv[++idx]);
    targetDir = std::string(argv[++idx]);
    
    bool success = true;
    if(argc>++idx)
	success = readOptArgs(argc, argv, idx);

    return success;
}


bool readOptArgs(int argc, char**argv, int offset){

    
    int search_idx = parse_argument (argc, argv, 
				     "-withPlaus", 
				     _use_plausibilities);
    
  return true;
}

void errorMsg(){
    fprintf(stderr,"Wrong arguments \n ./meshCloud pcdfile treeDepth targetDir [optional arguments]\n optional arguments: -withPlaus\n");
}

