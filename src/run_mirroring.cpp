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
#include <mirror_objects/parse.h>
#include <mirror_objects/predictObject.h>
#include <mirror_objects/utilities.h>
#include <libgen.h>

#include <mirror_objects/calibration.h>

#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#define BUFSIZE 8096
#define VOXSIZE 10
#define MINPTS 90
 
std::vector<cv::Point3f> mCloud;

cv::Point3d mOrig;
cv::Mat     mTrans;
StereoParameters mStereo;
cv::Point2d mFocal(500.0f, 500.0f);
cv::Point2d mCentre(320.0f, 240.0f);
std::string search_space = "FullSearch";
std::string targetDir;
cv::Mat     mMask;
int         mDepth = 5;

bool _no_rot_exists = false;

bool mHaveTrans = false;

// Functions for parsing the input arguments
bool parseParameters(int argc, char**argv);
bool readOptArgs(int argc, char**argv, int offset);
void errorMsg();

void DumpCenter( const char *filename, cv::Point3d center);

void DumpPlausibility( const char *filename, float plaus);

void dumpPCD(const char* name, sensor_msgs::PointCloud2 &result);

int main(int argc, char**argv){

    std::string newName;
    std::string origName;
  
    if(!parseParameters(argc, argv))
	exit(-1);
    else {
	// Print Parameters
	origName = std::string(basename(argv[1]));
	int outLength = origName.size()-4;
	newName = origName.substr(0,outLength);
    
	if(!mHaveTrans)
	    std::cout << "Viewpoint: " 
		      << mOrig.x << " " 
		      << mOrig.y << " " 
		      << mOrig.z << std::endl;
	else {
	    std::cout << "World to Camera Transformation: " << std::endl;
	    for(int r=0; r<3; r++){
		float* mTransi = mTrans.ptr<float>(r);
		for(int c=0;c<4;++c)
		    std::cout << "\t" << mTransi[c] ;
		std::cout << std::endl;
	    }
	}
    
	std::cout << "Target Directory: " << targetDir << std::endl;
	std::cout << "Output name: " << newName << std::endl;
	std::cout << "Tree Depth for approximation: " << mDepth << std::endl;
	std::cout << "Focal length: " << mFocal.x << " " << mFocal.y << std::endl;
	std::cout << "Image centre: " << mCentre.x << " " << mCentre.y << std::endl;
	if(!mMask.empty())
	    std::cout << "Have Image Mask." << std::endl;
    }
  
  
    // If filter is true, outliers are filtered in voxel grid as well as 
    // in table projection of point cloud.
    // In case of sparse point clouds set "filter" to false and "sparse" to true
    
    bool filter = true;
    bool sparse = false;

    if(filter){
	cv::Point3d leaf_size;
	leaf_size.x = VOXSIZE;
	leaf_size.y = VOXSIZE;
	leaf_size.z = VOXSIZE;
    
	cv::Point2f minmaxX, minmaxY, minmaxZ;
	GetMinMaxPoints( mCloud, minmaxX, minmaxY, minmaxZ ); 
	printf("MinMax of segment  X: %f %f  Y: %f %f  Z: %f %f\n", 
	       minmaxX.x, minmaxX.y,minmaxY.x, minmaxY.y, minmaxZ.x, minmaxZ.y);
	FilterSegment( mCloud,leaf_size,MINPTS,minmaxX,minmaxY,minmaxZ );
	printf("Number of points after filtering: %ld\n",(long int)mCloud.size());    
    }
  
    PredictObject *predObj;
    if(mHaveTrans)
	predObj=new PredictObject( mCloud, 0, mTrans, mDepth, 
				   sparse, mFocal, mCentre, mMask, 
				   targetDir.c_str(), newName.c_str());
    else 
	predObj=new PredictObject( mCloud, 0, mOrig, mDepth, 
				   sparse, mFocal, mCentre, mMask,
				   targetDir.c_str(), newName.c_str());
  
    if(_no_rot_exists){
	predObj->SetFullSearch(false);
	search_space = "ShiftSearch";
    }
    predObj->Mirror(filter);
  
    // Get Mean of original point cloud
    cv::Point3f altCenter;
    predObj->GetMeanAllAxis(altCenter);
    std::cout << "Orig Center " 
	      << altCenter.x << " " 
	      << altCenter.y << " " 
	      << altCenter.z << std::endl; 
    char filename[BUFSIZE];
    sprintf(filename, 
	    "%s/CenterNoMir%s_%s.txt",
	    targetDir.c_str(), 
	    search_space.c_str(),
	    newName.c_str());
    DumpCenter(filename, altCenter);
  
    // Get Mean of completed point cloud
    predObj->GetMirMeanAllAxis( altCenter );
    std::cout << "New Center from Mirrored Cloud " 
	      << altCenter.x << " " 
	      << altCenter.y << " " 
	      << altCenter.z << std::endl; 
  
    sprintf(filename, 
	    "%s/CenterMir%s_%s.txt",
	    targetDir.c_str(), 
	    search_space.c_str(),
	    newName.c_str());
    DumpCenter(filename, altCenter);

    // Get Mean ON TOP of original point cloud
    predObj->GetMean(altCenter);
    std::cout << "Orig Top Center " 
	      << altCenter.x << " " 
	      << altCenter.y << " " 
	      << altCenter.z << std::endl; 
    sprintf(filename, 
	    "%s/TopCenterNoMir%s_%s.txt",
	    targetDir.c_str(), 
	    search_space.c_str(),
	    newName.c_str());
    DumpCenter(filename, altCenter);

    // Get Mean ON TOP of completed point cloud
    predObj->GetMirMean( altCenter );
    std::cout << "New Top Center from Mirrored Cloud " 
	      << altCenter.x << " " 
	      << altCenter.y << " " 
	      << altCenter.z << std::endl; 
  
    sprintf(filename, 
	    "%s/TopCenterMir%s_%s.txt",
	    targetDir.c_str(), 
	    search_space.c_str(),
	    newName.c_str());
    DumpCenter(filename, altCenter);

    // get parameters of the mirroring plane
    sprintf(filename, 
	    "%s/Plane_%s_%s.txt",
	    targetDir.c_str(), 
	    search_space.c_str(),
	    newName.c_str());
    predObj->DumpPlaneParams(filename);
  
    // get the mirrored point cloud to dump it as a pcd file
    //ros::init(argc, argv, "run_mirroring");
    //ros::NodeHandle nh;
    sensor_msgs::PointCloud2 mirrored_cloud;
    std::vector<cv::Point3f> point_vec;
    std::vector<cv::Point3f> normals;
    std::vector<float> plausibilities;
    std::vector<bool> original;

    predObj->GetMirCloud(point_vec);
    predObj->GetNormals(normals);
    predObj->GetPlausibility(plausibilities);
    predObj->GetOriginal(original);


    // construct PointCloud2
    mirrored_cloud.header.frame_id = "/narrow_stereo_optical_frame";
    //mirrored_cloud.header.stamp = ros::Time::now();
    mirrored_cloud.width        = point_vec.size();
    mirrored_cloud.height       = 1;
    mirrored_cloud.is_dense     = true;
    mirrored_cloud.is_bigendian = false;
    // size of field is 3 for x, y, z, 1 for rgb/plausability and
    // 3 for normals and 1 for curvature and 1 for indicating if
    // from original point cloud or not
    mirrored_cloud.fields.resize( 3 + 1 + 3 + 1 + 1);
    mirrored_cloud.fields[0].name = "x"; 
    mirrored_cloud.fields[1].name = "y"; 
    mirrored_cloud.fields[2].name = "z";
    mirrored_cloud.fields[3].name = "rgb";
    mirrored_cloud.fields[4].name = "normal_x"; 
    mirrored_cloud.fields[5].name = "normal_y"; 
    mirrored_cloud.fields[6].name = "normal_z";
    mirrored_cloud.fields[7].name = "curvature";
    mirrored_cloud.fields[8].name = "origin";
    int offset = 0;
    for (size_t d = 0; 
	 d < mirrored_cloud.fields.size (); 
	 ++d, offset += sizeof(float)) {
	mirrored_cloud.fields[d].offset = offset;
	mirrored_cloud.fields[d].datatype = 
	    sensor_msgs::PointField::FLOAT32;
	mirrored_cloud.fields[d].count  = 1;
    }
    
    mirrored_cloud.point_step = offset;
    mirrored_cloud.row_step   = 
	mirrored_cloud.point_step * mirrored_cloud.width;
    
    mirrored_cloud.data.resize (mirrored_cloud.width * 
				mirrored_cloud.height * 
				mirrored_cloud.point_step);
    
    float curvature = 0.0f;
    float hypo = 0, orig = 255;

    for(unsigned int i=0; i < point_vec.size(); i++){
	memcpy (&mirrored_cloud.data[i * mirrored_cloud.point_step + mirrored_cloud.fields[0].offset], 
		&point_vec[i].x, sizeof (float));
	memcpy (&mirrored_cloud.data[i * mirrored_cloud.point_step + mirrored_cloud.fields[1].offset], 
		&point_vec[i].y, sizeof (float));
	memcpy (&mirrored_cloud.data[i * mirrored_cloud.point_step + mirrored_cloud.fields[2].offset], 
		&point_vec[i].z, sizeof (float));
	memcpy (&mirrored_cloud.data[i * mirrored_cloud.point_step + mirrored_cloud.fields[3].offset], 
		&plausibilities[i], sizeof (float));
	memcpy (&mirrored_cloud.data[i * mirrored_cloud.point_step + mirrored_cloud.fields[4].offset], 
		&normals[i].x, sizeof (float));
	memcpy (&mirrored_cloud.data[i * mirrored_cloud.point_step + mirrored_cloud.fields[5].offset], 
		&normals[i].y, sizeof (float));
	memcpy (&mirrored_cloud.data[i * mirrored_cloud.point_step + mirrored_cloud.fields[6].offset], 
		&normals[i].z, sizeof (float));
	memcpy (&mirrored_cloud.data[i * mirrored_cloud.point_step + mirrored_cloud.fields[7].offset], 
		&curvature, sizeof (float));
	if(original[i])
	    memcpy (&mirrored_cloud.data[i * mirrored_cloud.point_step + mirrored_cloud.fields[8].offset], 
		    &orig, sizeof (float));
	else 
	    memcpy (&mirrored_cloud.data[i * mirrored_cloud.point_step + mirrored_cloud.fields[8].offset], 
		    &hypo, sizeof (float));
    }
  
    sprintf(filename, 
	    "%s/Mirrored%s_%s.pcd",
	    targetDir.c_str(), 
	    search_space.c_str(),
	    newName.c_str());
    dumpPCD(filename, mirrored_cloud);
  
    // Store plausibility 
    float plausibility;
    predObj->GetPlausibility(plausibility);
    sprintf(filename, 
	    "%s/Plausibility%s_%s.txt", 
	    targetDir.c_str(), 
	    search_space.c_str(),
	    newName.c_str());
    DumpPlausibility(filename,plausibility);
  
}

bool parseParameters(int argc, char**argv){
    
    if( argc<6 || argc>13 ||
	( (argc<8 || argc>13) && std::string(argv[2])=="-p") || 
	( (argc<6 || argc>11) && 
	  (std::string(argv[2])=="-v" || std::string(argv[2])=="-t") ) ) 
    {
	errorMsg();
	return false;
    }
    
    int i=0;
    
    parsePoints(argv[++i],mCloud);
    printf("Number of points: %ld\n", (long int)mCloud.size());
    
    if(mCloud.size()==0)
      return false;

    i++;
    
    if(std::string(argv[i])=="-p"){
	mOrig.x = atof(argv[++i]);
	mOrig.y = atof(argv[++i]);
	mOrig.z = atof(argv[++i]);
	
    } else if(std::string(argv[i])=="-v") {
	std::ifstream ifs(argv[++i]);
	if(ifs.good()){
	    std::string line;
	    getline(ifs,line, ',');
      
	    if(line[0]=='('){
	
		std::string num = line.substr(1,line.size()-1);
		mOrig.x = atof(num.c_str());
	
		getline(ifs,line, ',');
		mOrig.y = atof(line.c_str());
	
		getline(ifs,line, ')');
		mOrig.z = atof(line.c_str());
	
	    } else {
		std::stringstream ss(line);
		ss >> mOrig.x >> mOrig.y >> mOrig.z;
	    }
	    
	} else {
	    fprintf(stderr, "1.%s cannot be read as a file.\n", argv[i]);
	    exit(-1);
	}
    } else if(std::string(argv[i])=="-t") {
	std::ifstream ifs(argv[++i]);
	if(ifs.good()){
	    
	    std::string line;
	    int rows=3, cols=4;
	    int r=0;
	    mTrans = cv::Mat(rows, cols, CV_32F);
	    mTrans.setTo(0.0f);
	    
	    while ( !ifs.eof() && r<rows){
		getline (ifs,line);
		std::stringstream ss(line);
		float* mTransi = mTrans.ptr<float>(r);
		for(int c=0;c<cols;++c)
		    ss >> mTransi[c];
		r++;
		
	    }
	    
	    if(r<3){
		fprintf(stderr, "%s does not contain a 3x4 transformation matrix.\n", argv[i]);
		exit(-1);
	    }
	    
	    mHaveTrans = true;
	    
	} else {
	    fprintf(stderr, "2.%s cannot be read as a file.\n", argv[i]);
	    exit(-1);
	}
    } else {
	fprintf(stderr, "%s seems to be an invalid option.\n", argv[i]);
	exit(-1);
    }
    
    mDepth = atof(argv[++i]);
    targetDir = std::string(argv[++i]);
    
    bool success = true;
    if(argc>++i)
	success = readOptArgs(argc, argv, i);
    
    return success;
}


bool readOptArgs(int argc, char**argv, int offset){

    
  bool cam_exists = false;
  int cam_idx = parse_argument (argc, argv, "-f", cam_exists);
  if(cam_exists)
    {
      /*
	std::ifstream ifs(argv[cam_idx+1]);
	std::string line;
	if(!ifs.is_open()){
	    fprintf(stderr, "3.%s cannot be read as a file.\n",
		    argv[cam_idx+1] );
	    exit(-1);
	}
	
	getline(ifs,line);
	std::stringstream ss1(line);
	ss1 >> mFocal.x >> mFocal.y;
      
	getline(ifs,line);
	std::stringstream ss2(line);
	ss2 >> mCentre.x >> mCentre.y;
      */

      	std::ifstream ifs(argv[cam_idx+1]);
	ifs >> mStereo;
	mFocal = mStereo.mLeftInt.GetF();
	mCentre = mStereo.mLeftInt.GetC();
    }
  
  bool mask_exists = false;
  int mask_idx = parse_argument (argc, argv, "-m", mask_exists);
  if(mHaveTrans && mask_exists){
      std::string maskFile = std::string(argv[mask_idx+1]);
      mMask = cv::imread(maskFile,0); 
      
      if( !mMask.data ) {
	  fprintf(stderr, "4.%s cannot be read as a file.\n", maskFile.c_str());
	  exit(-1);
      }
  } else if(mask_exists) {
      std::cout << "* Mask image only useful with precise camera parameters. Will not be used." << std::endl;
  }
  
  int search_idx = parse_argument (argc, argv, 
				   "-noRot", 
				   _no_rot_exists);

  return true;
}

void errorMsg(){
    fprintf(stderr,"Wrong arguments \n ./mirrorCloud crdfile -p viewx viewy viewz treeDepth targetDir [optional arguments]\n or ./mirrorCloud crdfile -v viewpointFile     treeDepth targetDir [optional arguments]\n or ./mirrorCloud crdfile -t world2CamFile     treeDepth targetDir [optional arguments]\n optional arguments:\n -f cameraCalibration \n -m maskFile\n -noRot\n");
}

void DumpCenter( const char *filename, cv::Point3d center)
{
    FILE *lFILE=fopen(filename,"w");
    fprintf(lFILE,"%f %f %f\n", center.x, center.y, center.z);
    fclose(lFILE);
}

void DumpPlausibility( const char *filename, float plaus)
{
    FILE *lFILE=fopen(filename,"w");
    fprintf(lFILE,"%f\n", plaus);
    fclose(lFILE);
}


void dumpPCD(const char* name, sensor_msgs::PointCloud2 &result)
{

    int w = result.width;
    int h = result.height;

    float min = 1.0f;
    float max = 0.0f;

    for(int y=0; y<h; ++y)
    {
	for(int x=0; x<w; ++x)
	{
	    int i = y*w+x;
	    float plaus;
	  
	    memcpy(&plaus, 
		   &result.data[i*result.point_step+result.fields[3].offset],
		   sizeof (float));

	    if(plaus>max)
		max = plaus;
	    else if(plaus<min)
		min = plaus;
	  
	    float rgb = getRGB(plaus, plaus, plaus);

	    memcpy(&result.data[i*result.point_step+result.fields[3].offset],
		   &rgb, 
		   sizeof (float));
	  
	}
    }
  
    pcl::PCDWriter writer;
    pcl::PCLPointCloud2 result_tmp;
    pcl_conversions::copyPointCloud2MetaData(result, result_tmp);
    writer.writeBinary(name, result_tmp);

    // std::cout << "Min and max of plausability " << min << " " << max << std::endl;

}
