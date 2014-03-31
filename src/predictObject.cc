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
#include "predictObject.h"
#include "quat.hpp"
#include "utilities.h"
#include <opencv2/highgui/highgui.hpp>
#include <fstream>
#include <iostream>

#include <sys/time.h>



#define BUFSIZE 512

#define PIXELTHRESH 1

#define HYPOS 5
#define ANGLEHYPOS 6
#define MINMAXANGLE 20

#define ROWS 480
#define COLS 640

#define GLOBALMIN 1
#define MEDIAN 0

using namespace std;
 
PredictObject::PredictObject() 
{
  eigenVectors = 0;
  eigenValues = 0;
  avg = 0;

  mDirName = 0;
  mNoDemo = 0;
};

PredictObject::PredictObject( std::vector<cv::Point3f> lCloud,
			      int id,
			      cv::Point3f lOrig,
			      int lDepth,
			      bool lSparse,
			      cv::Point2d lFocal,
			      cv::Point2d lCentre,
			      cv::Mat lMask,
			      const char* lDirName,
			      const char* lNoDemo)
  : objID(id),
    orig(lOrig),
    maxDiverge(0),
    nDiverge(0),
    maxOverlap(0),
    nOverlap(0),
    maxHidden(0),
    nHidden(0),
    bestVote(1.0f),
    mHavePredicted(false),
    mHaveMirrored(false),
    mHaveTrans(false),
    mUseConfidence(false),
    full_search_(true),
    mSparse(lSparse),
    mDepth(lDepth),
    mFocal(lFocal),
    mCentre(lCentre),
    orig_plaus(0.95)
 {
  mCloud.clear();
  mCloud=lCloud;

  if(!lMask.empty()){
    mMask = lMask;
  }

  eigenVectors = cvCreateMat( 2, 2, CV_32FC1 );   
  eigenValues  = cvCreateMat( 1, 2, CV_32FC1 );   

  avg = cvCreateMat( 1, 2, CV_32FC1 );   

  mDirName = (char *)malloc(BUFSIZE*sizeof(char));
  sprintf(mDirName, "%s", lDirName);
  mNoDemo = (char *)malloc(BUFSIZE*sizeof(char));
  sprintf(mNoDemo, "%s", lNoDemo);

  
}
  
PredictObject::PredictObject( std::vector<cv::Point3f> lCloud,
			      int id,
			      cv::Mat lTrans,
			      int lDepth,
			      bool lSparse, 
			      cv::Point2d lFocal,
			      cv::Point2d lCentre,
			      cv::Mat lMask,
			      const char* lDirName,
			      const char* lNoDemo)
  : objID(id),
    mTrans(lTrans),
    maxDiverge(0),
    nDiverge(0),
    maxOverlap(0),
    nOverlap(0),
    maxHidden(0),
    nHidden(0),
    bestVote(1.0f),
    mHavePredicted(false),
    mHaveMirrored(false),
    mHaveTrans(true),
    mUseConfidence(false),
    full_search_(true),
    mSparse(lSparse),
    mDepth(lDepth),
    mFocal(lFocal),
    mCentre(lCentre),
    orig_plaus(0.95)
{
  mCloud.clear();
  mCloud=lCloud;
  
  if(!lMask.empty())
    mMask = lMask;
  
  orig.x = mTrans.at<float>(0,3);
  orig.y = mTrans.at<float>(1,3);
  orig.z = mTrans.at<float>(2,3);

  eigenVectors = cvCreateMat( 2, 2, CV_32FC1 );   
  eigenValues  = cvCreateMat( 1, 2, CV_32FC1 );   

  avg = cvCreateMat( 1, 2, CV_32FC1 );   

  mDirName = (char *)malloc(BUFSIZE*sizeof(char));
  sprintf(mDirName, "%s", lDirName);
  mNoDemo = (char *)malloc(BUFSIZE*sizeof(char));
  sprintf(mNoDemo, "%s", lNoDemo);
}


PredictObject::PredictObject( std::vector<cv::Point3f> lCloud,
			      std::vector<cv::Point3f> normals,
			      std::vector<float> plausibilities,
			      int id,
			      int lDepth,
			      bool use_plausibilities,
			      const char* lDirName,
			      const char* lNoDemo)
  : objID(id)
  , mHavePredicted(true)
  , mHaveMirrored(false)
  , mUseConfidence(use_plausibilities)
  , full_search_(true)
  , mDepth(lDepth)

{
  mMirCloud.clear();
  mMirCloud=lCloud;
  mNormals.clear();
  mNormals=normals;
  mPlausibilities.clear();
  mPlausibilities=plausibilities;

  eigenVectors = 0;
  eigenValues = 0;
  avg = 0;

  mDirName = (char *)malloc(BUFSIZE*sizeof(char));
  sprintf(mDirName, "%s", lDirName);
  mNoDemo = (char *)malloc(BUFSIZE*sizeof(char));
  sprintf(mNoDemo, "%s", lNoDemo);
}

PredictObject::~PredictObject()
{
  if(eigenVectors!=0)
    cvReleaseMat(&eigenVectors);
  
  if(eigenValues!=0)
    cvReleaseMat(&eigenValues);
  
  if(avg!=0)
    cvReleaseMat(&avg);

  if(mDirName!=0)
    free(mDirName);
  
  if(mNoDemo!=0)
    free(mNoDemo);
}

bool PredictObject::HavePredicted()
{
  return mHavePredicted;
}

bool PredictObject::HaveMirrored()
{
  return mHaveMirrored;
}

int PredictObject::GetID()
{
  return objID;
}

bool PredictObject::GetMirCloud(vector<cv::Point3f> &lMirCloud)
{
  if(mHaveMirrored){
    lMirCloud.clear();
    lMirCloud = mMirCloud;
    return true;
  } else 
    return false;
}


void PredictObject::Mirror( bool filter )
{
  vector<cv::Point3f>  lab_cloud;
  vector<cv::Point3f>  mir_cloud;
  
  cv::Point2f minmaxX, minmaxY, minmaxZ;
  GetMinMaxPoints(mCloud, minmaxX, minmaxY, minmaxZ);
  minX=minmaxX.x;
  maxX=minmaxX.y;
  minY=minmaxY.x;
  maxY=minmaxY.y;
  minZ=minmaxZ.x;
  maxZ=minmaxZ.y;
  
  cout << "Min & Max in X: " << minX << " " << maxX
       << " Min & Max in Y:" << minY << " " << maxY << endl;
  
  cv::Mat lProj;
  ProjectOnTable( mCloud, lProj, minX, maxX, minY, maxY, filter );
  mProj.create(lProj.size(),lProj.type());
  mProj=lProj;
  
  // Get eigenvectors for first mirroring plane approximation
  CalcPCA();
  cv::Vec3f avg3D(avg->data.fl[0]+minX, 
		  avg->data.fl[1]+minY, 
		  minZ+(maxZ-minZ)/2.0f);
  
  // Prepare mask image, depth look-up image and set of points on which 
  // KNN index is computed
  cv::Mat lPoints;
  cv::flann::Index lFLANNIdx = CreateLookUpData(mCloud, 
						avg3D, 
						mDepthLU, 
						mMask, 
						lPoints);
    
  std::cout << "Determine Mirroring Parameters ..." << std::endl;
  vector<cv::Point3f> normals;
  vector<float> plausibilities;
  vector<bool> original;
  
  if(full_search_)
  {
      MirrorCloudSearchOrient(lPoints, 
			      lFLANNIdx, 
			      avg3D, 
			      lab_cloud, 
			      mir_cloud, 
			      normals, 
			      plausibilities,
			      original);
  } 
  else {
      
      MirrorCloud(lPoints, 
		  lFLANNIdx, avg3D, 
		  lab_cloud, mir_cloud, 
		  normals, plausibilities,
		  original);
  }
  
  mMirCloud = mir_cloud;
  mNormals = normals;
  mPlausibilities = plausibilities;
  mOriginal = original;
  mHaveMirrored = true;
  
  
  char name[512];
  sprintf(name, "mirrored.crd");
  DumpMirroredCloud(name, mMirCloud, plausibilities);
  
  sprintf(name, "orig.crd");
  DumpMirroredCloud(name, mCloud);

}

cv::flann::Index PredictObject::CreateLookUpData(const std::vector<cv::Point3f> &lCloud,
						 const cv::Vec3f avg3D,
						 cv::Mat &lDepth, 
						 cv::Mat &lMask, 
						 cv::Mat &lPoints)
{
  MakeDepthMap(lCloud, avg3D, lDepth, lMask, lPoints);

  //  Fill in holes within mask (for sparse make convex hull)
  FillProjection(lMask, lMask, mSparse);
  std::cout << "Making the KNN Index ..." << std::endl;
  return MakeFLANNIndex( lPoints);

}

void PredictObject::ConstructMesh( )
{
  
    try 
    {	
	if(mUseConfidence) 
	{
	    std::cout << "Attempting mesh reconstruction exploiting plausibilities" << std::endl;
	    ApproximateMesh(mMirCloud,mNormals,mPlausibilities);
	} 
	else 
	{
	    std::cout << "Attemtping Mesh Reconstruction with every point weighted equally" << std::endl;
	    std::vector<float> empty;
	    ApproximateMesh(mMirCloud,mNormals,empty);
	}
    } 
    catch(...) 
    {
	cout << "Approximating Meshes failed " << endl;
    }
}

void PredictObject::ProjectOnTable( vector<cv::Point3f> &lCloud, 
				    cv::Mat &lProj, 
				    float minX, float maxX, 
				    float minY, float maxY, 
				    bool filter )
{
    std::vector<Leaf>leaves;
    std::vector<int> indices2;
    indices2.reserve(lCloud.size());
    indices2.resize(lCloud.size(),0);
    for(unsigned int k=0; k < lCloud.size(); k++)
	indices2.at(k)=k;
  
    float distX = maxX - minX;
    float distY = maxY - minY;
  
    // Allocate the space needed
    try 
    {
	if (leaves.capacity () < (distY+1)*(distX+1))
	    leaves.reserve ((distX+1)*(distY+1));             
	leaves.resize ((distX+1)*(distY+1));
    } 
    catch (std::bad_alloc) 
    {
	printf("Failed while attempting to allocate a vector of %f (%g x %g) leaf elements (%f bytes total)", 
	       (distY+1)*(distX+1), distX+1, distY+1, (distY+1)*(distX+1)*sizeof (Leaf));
    }

    unsigned long leaves_size = (distY+1)*(distX+1);
    if (leaves_size != leaves.size ())
	printf("That's odd: %lu != %zu", leaves_size, leaves.size());

    // image left handed while robot right handed 
    // (therefore flipping of coordinates)
    lProj.create(distX+1, distY+1,CV_8U);
    lProj.setTo(0);

    cout << "ProjectOnTable started ... " << endl;
  
    float x, y; 
    int cellX, cellY;
    int cp=0;
    //calculate cells in a loop over all 3D Points
    for ( vector<cv::Point3f>::iterator i = lCloud.begin(); 
	  i < lCloud.end(); ++i) 
    {    
	x = i->x;
	y = i->y;
      
	//calculate x and y cell
	cellX = x-minX;
	cellY = y-minY;

	lProj.at<uchar>(cellX, cellY) = lProj.at<uchar>(cellX, cellY)+1;
	int idx = cellY*(distX+1)+cellX;
	leaves[idx].pts.push_back(cp);
      
	cp++;
      
    }
  
    if( filter ){

	cout << "Preprocessing Grid ... " << endl;
	cv::GaussianBlur(lProj,lProj,cv::Size(3,3),1.0f);
	cv::threshold(lProj, lProj, PIXELTHRESH, 255,cv::THRESH_BINARY);
    }
} 

void PredictObject::CalcPCA( bool calcCH )
{
  
  int width = mProj.size().width;
  int height = mProj.size().height;

  int k = cv::countNonZero(mProj);

  CvMat* src = cvCreateMat( k, 2, CV_32FC1 );   
  int step = src->step/sizeof(float);     
  
  
  CvPoint* points = (CvPoint*)malloc( k * sizeof(CvPoint));
  CvMat pointMat = cvMat( 1, k, CV_32SC2, points );
  CvPoint pt0;
  
  int count = 0;
  float *data = src->data.fl;   
  for (int y=0;y<width;y++){
    for (int x=0;x<height;x++){
      if(mProj.at<uchar>(x,y)!=0){
	(data+count*step)[0]=x;
	(data+count*step)[1]=y;
	
	pt0.x = x;
	pt0.y = y;
	points[count] = pt0;
	
	count++;
      }
    }
  }   
  
  cvCalcPCA( src, avg, eigenValues, eigenVectors, CV_PCA_DATA_AS_ROW);   
  
  cout << "eigenVectors " << eigenVectors->data.fl[0] 
       << " " <<  eigenVectors->data.fl[1] 
       << " " <<  eigenVectors->data.fl[2] 
       << " " <<  eigenVectors->data.fl[3] 
       << endl;

  cout << "mean " << avg->data.fl[0] << " " << avg->data.fl[1] << endl;

  // calc convex hull and use its center instead of mean of projection
  if(calcCH){
    int* hull = (int*)malloc( k * sizeof(int));
    CvMat hullMat = cvMat( 1, k, CV_32SC1, hull );
    cvConvexHull2( &pointMat, &hullMat, CV_CLOCKWISE, 0 );
    int hullcount = hullMat.cols;
    
    cv::Mat rgbProj;
    rgbProj.create(mProj.size(),CV_8UC3);
    cv::cvtColor(mProj,rgbProj,CV_GRAY2RGB);
    rgbProj = rgbProj.mul(cv::Mat::ones(rgbProj.rows, rgbProj.cols,CV_8UC3), 255.0f);
    
    pt0 = points[hull[0]];
    float avgX = pt0.x;
    float avgY = pt0.y;
    
    CvPoint pt0_S;
    pt0_S.x = pt0.y;
    pt0_S.y = pt0.x;
    
    for (int i=1;i<hullcount;i++){
      CvPoint pt = points[hull[i]];
      avgX += pt.x;
      avgY += pt.y;
      
      CvPoint pt_S;
      pt_S.x = pt.y;
      pt_S.y = pt.x;

      cv::line( rgbProj,pt0_S, pt_S, CV_RGB( 0, 255, 0 ));
      pt0 = pt;
      pt0_S = pt_S;
    }
    
    pt0 = points[hull[0]];
    CvPoint pt0_End;
    pt0_End.x = pt0.y;
    pt0_End.y = pt0.x;
    cv::line( (rgbProj),pt0_S, pt0_End, CV_RGB( 0, 255, 0 ));
    
    avgX /= hullcount;
    avgY /= hullcount;
    
  CvPoint avgCH;
  avgCH. x =  avgY;
  avgCH. y =  avgX;
  cv::circle(rgbProj, avgCH, 10, CV_RGB( 0, 0, 255 ));
  avgCH. x =  avg->data.fl[1];
  avgCH. y =  avg->data.fl[0];
  cv::circle(rgbProj, avgCH, 10, CV_RGB( 255, 0, 0 ));
  
  //  cv::imwrite("convexHull.pgm", rgbProj);

  // using centre of convex hull for mirroring instead
  avg->data.fl[0] = avgX;
  avg->data.fl[1] = avgY;
  
  free(hull);

  }
  
  cvReleaseMat(&src);
  free(points);
}

void PredictObject::CalcCentroid(const cv::Mat &lProj, cv::Point2d &lCentroid)
{
  CvMat* lEigenVectors;
  CvMat* lEigenValues;
  CvMat* lAvg;

  lEigenVectors = cvCreateMat( 2, 2, CV_32FC1 );   
  lEigenValues  = cvCreateMat( 1, 2, CV_32FC1 );   
  lAvg = cvCreateMat( 1, 2, CV_32FC1 );   

  int width  = lProj.size().width;
  int height = lProj.size().height;

  int k = cv::countNonZero(lProj);

  CvMat* src = cvCreateMat( k, 2, CV_32FC1 );   
  int step = src->step/sizeof(float);     
 
  /*
  CvPoint* points = (CvPoint*)malloc( k * sizeof(CvPoint));
  CvMat pointMat = cvMat( 1, k, CV_32SC2, points );
  CvPoint pt0;
  */
  int count = 0;
  
  float *data = src->data.fl;   
  for (int y=0;y<width;y++){
    for (int x=0;x<height;x++){
      if(lProj.at<uchar>(x,y)!=0){
	(data+count*step)[0]=x;
	(data+count*step)[1]=y;

	/*
	pt0.x = x;
	pt0.y = y;
	points[count] = pt0;
	*/
	count++;
	
      }
    }
  }   
  
  //  cout << "this two values should be the same " << count << " " << k << endl;

  cvCalcPCA( src, lAvg, lEigenValues, lEigenVectors, CV_PCA_DATA_AS_ROW);   

  lCentroid.x = lAvg->data.fl[0];
  lCentroid.y = lAvg->data.fl[1];


  cvReleaseMat(&lEigenVectors);
  cvReleaseMat(&lEigenValues);
  cvReleaseMat(&lAvg);
  cvReleaseMat(&src);
  
}


void PredictObject::CalcCentroid(const vector<cv::Point3f> lCloud, cv::Point3f &lAvg)
{
  float xAvg=0, yAvg=0, zAvg=0;
  for(vector<cv::Point3f>::const_iterator it=lCloud.begin();it!=lCloud.end();++it) {
    xAvg+=it->x;
    yAvg+=it->y;
    zAvg+=it->z;
  } 
  xAvg/=lCloud.size();
  yAvg/=lCloud.size();
  zAvg/=lCloud.size();
  
  lAvg.x=xAvg;
  lAvg.y=yAvg;
  lAvg.z=zAvg;

}


void PredictObject::MakeDepthMap( const std::vector<cv::Point3f> &lCloud, 
				  const cv::Vec3f avg3D, 
				  cv::Mat &lDepth, 
				  cv::Mat &lMask, 
				  cv::Mat &lPoints)
{
  lDepth.create(ROWS, COLS,CV_32F);
  lDepth.setTo(0);
  
  if(mHaveTrans){
    
    CV_Assert(mTrans.rows == 3 && mTrans.cols == 4);
    cv::Mat rotMat = mTrans.colRange(0,mTrans.cols-1);
    ProjectPoints(lCloud, rotMat, lDepth, lMask, lPoints);
    
    cv::imwrite("transMask.pgm", lMask);


  } else {
    
    cv::Vec3f lViewDir = avg3D - cv::Vec3f(orig.x, orig.y, orig.z);
    Quat rot,rot2;
    rot.makeRotate(cv::Vec3f(0,0,1),lViewDir);
    rot2.makeRotate(cv::Vec3f(0.45,-0.55,0),cv::Vec3f(0,1,0));
    ProjectPoints(lCloud, rot2*rot, lDepth, lMask, lPoints);
    
    cv::imwrite("viewMask.pgm", lMask);

  }
}

cv::flann::Index PredictObject::MakeFLANNIndex(const cv::Mat &lPoints)
{
  cv::flann::Index flann_index(lPoints, cv::flann::KDTreeIndexParams(4));
  return flann_index;
}


void PredictObject::ProjectOnImage( std::vector<cv::Point3f> &lCloud, 
				    const cv::Mat &lMask, 
				    const cv::Vec3f avg3D, 
				    cv::flann::Index *lFLANNIdx, 
				    const cv::Mat &lPoints,
				    cv::Mat &lImg, 
				    cv::Mat &lOver, 
				    cv::Mat &lDepth,
				    std::vector<std::pair<bool,float> > &lPlaus)
{
  maxDiverge = 0;
  maxOverlap = 0;
  maxHidden  = 0;

  std::vector<cv::Point3f> newCloud;
  lPlaus.clear();

  lImg.create(ROWS, COLS,CV_32F);
  lImg.setTo(0);
  lOver.create(ROWS, COLS,CV_32F);
  lOver.setTo(0);

  // create images that count points projected to the pixels
  cv::Mat lNImg(ROWS, COLS, CV_32S);
  lNImg.setTo(0);
  cv::Mat lNOver(ROWS, COLS, CV_32S);
  lNOver.setTo(0);
  
  float max_x=0, max_y=0, max_z=0;
  float min_x=1000, min_y=1000, min_z=1000;

  if(mHaveTrans){
    CV_Assert(mTrans.rows == 3 && mTrans.cols == 4);
    cv::Mat rotMat = mTrans.colRange(0,mTrans.cols-1);

    cv::Mat lRotCloudTrans;
    Arm2Cam(lCloud, rotMat, lRotCloudTrans);
    
    for(int i=0; i<lRotCloudTrans.rows; ++i){
      const float* Ri = lRotCloudTrans.ptr<float>(i);
      
      float x = (Ri[0]*mFocal.x)/Ri[2]+mCentre.x;
      float y = (Ri[1]*mFocal.y)/Ri[2]+mCentre.y;

      if(x>max_x)
	  max_x = x;
      else if(x<min_x)
	  min_x = x;

      if(y>max_y)
	  max_y = y;
      else if(y<min_y)
	  min_y = y;

      if(Ri[2]>max_z)
	  max_z = Ri[2];
      else if(Ri[2]<min_z)
	  min_z = Ri[2];


      std::pair<bool,float> plaus;
      /*      if( !ProjectPoint(x, y, Ri[2], lMask, 
			lFLANNIdx, lPoints, lImg, lOver, lDepth, plaus))
      */
      if( !ProjectPoint(x, y, Ri[2], lMask, 
			lFLANNIdx, lPoints, lImg, lOver, 
			lNImg, lNOver, lDepth, plaus))
	{
	  // overlapping points will not be included
	  newCloud.push_back(lCloud[i]);
	  lPlaus.push_back(plaus);
	}
    }
    
    
  } else {
   
    cv::Vec3f lViewDir = avg3D - cv::Vec3f(orig.x, orig.y, orig.z);
    Quat rot,rot2;
    rot.makeRotate(cv::Vec3f(0,0,1),lViewDir);
    rot2.makeRotate(cv::Vec3f(0.45,-0.55,0),cv::Vec3f(0,1,0));
    rot=rot2*rot;

    for(std::vector<cv::Point3f>::iterator it=lCloud.begin();
	it!=lCloud.end();++it) {
      
      float x, y, z;
      Arm2Cam(*it, rot, &x, &y, &z);

      if(x>max_x)
	  max_x = x;
      else if(x<min_x)
	  min_x = x;

      if(y>max_y)
	  max_y = y;
      else if(y<min_y)
	  min_y = y;

      if(z>max_z)
	  max_z = z;
      else if(z<min_z)
	  min_z = z;

      std::pair<bool,float> plaus;
      /*
      if( !ProjectPoint(x, y, z, lMask, lFLANNIdx, 
			lPoints, lImg, lOver, lDepth, plaus))
      */
      if( !ProjectPoint(x, y, z, lMask, lFLANNIdx, 
			lPoints, lImg, lOver, 
			lNImg, lNOver, lDepth, plaus))
	{
	  // overlapping points will not be included
	  newCloud.push_back(*it);
	  lPlaus.push_back(plaus);
	}
    }


  }
  
  // Normalise each pixel in lImg and lOver to number of 
  // points projected onto that pixel
  for (int y=0;y<lImg.cols;y++){
    for (int x=0;x<lImg.rows;x++){
      if(lImg.at<float>(x,y)!=0){
	assert(lNImg.at<int>(x,y)!=0);
	lImg.at<float>(x,y) = lImg.at<float>(x,y)/
	  (float)lNImg.at<int>(x,y);
      }
    }
  }

  for (int y=0;y<lOver.cols;y++){
    for (int x=0;x<lOver.rows;x++){
      if(lOver.at<float>(x,y)!=0){
	assert(lNOver.at<int>(x,y)!=0);
	lOver.at<float>(x,y) = lOver.at<float>(x,y)/
	  (float)lNOver.at<int>(x,y);
      }
    }
  }
  
 

  lCloud = newCloud;
}

void PredictObject::ProjectPoints( const std::vector<cv::Point3f> &lCloud,
				   const cv::Mat lRotMat, 
				   cv::Mat &lDepth, cv::Mat &lMask, 
				   cv::Mat &lPoints)
{
  lDepth.create(ROWS, COLS,CV_32F);
  lDepth.setTo(0);

  bool lCreateMask = false;
  if(lMask.empty()){
    lMask.create(ROWS, COLS,CV_8U);
    lMask.setTo(0);
    lCreateMask = true;
  }
  
  cv::Mat lRotCloudTrans;
  Arm2Cam(lCloud, lRotMat, lRotCloudTrans);
   
  std::vector<cv::Point2f> lPointVec;

  for(int i=0; i<lRotCloudTrans.rows; ++i){
     const float* Ri = lRotCloudTrans.ptr<float>(i);

     float x = (Ri[0]*mFocal.x)/Ri[2]+mCentre.x;
     float y = (Ri[1]*mFocal.y)/Ri[2]+mCentre.y;
     
     if(y<ROWS && y>0 && x<COLS && x>0){
       lDepth.at<float>(round(y), round(x)) = (float)Ri[2];
       cv::Point2f tmp(round(y), round(x));
       lPointVec.push_back(tmp);
       if(lCreateMask)
	 lMask.at<uchar>(round(y), round(x)) = 255;
     }
  }
  
  lPoints = cv::Mat(lPointVec, true).reshape(1);

}

void PredictObject::Arm2Cam(const std::vector<cv::Point3f> &lCloud, 
			    const cv::Mat lRotMat, cv::Mat &lRotCloudTrans)
{
  cv::Mat    v(3,1,CV_32F);
  v.at<float>(0,0)=orig.x;
  v.at<float>(1,0)=orig.y;
  v.at<float>(2,0)=orig.z;
  
  cv::Mat lCloudMat = cv::Mat(lCloud).reshape(1).t();
  cv::Mat lRotCloud(lCloudMat.size(),lCloudMat.type());
  
  for(int i=0; i<lCloudMat.cols; ++i){
    lCloudMat.col(i) -= v;
  }
  
  gemm(lRotMat, lCloudMat, 1.0f, 
       cv::Mat::zeros(lCloudMat.size(),lCloudMat.type()), 1.0f, 
       lRotCloud, cv::GEMM_1_T);
  
  lRotCloudTrans = lRotCloud.t();
}

void PredictObject::ProjectPoints( const std::vector<cv::Point3f> &lCloud,
				   const Quat rot, 
				   cv::Mat &lDepth, 
				   cv::Mat &lMask, 
				   cv::Mat &lPoints)
{
  lDepth.create(ROWS, COLS,CV_32F);
  lDepth.setTo(0);
  
  bool lCreateMask = false;
  if(lMask.empty()){
    lMask.create(ROWS, COLS,CV_8U);
    lMask.setTo(0);
    lCreateMask = true;
  }

  std::vector<cv::Point2f> lPointVec;

  for(std::vector<cv::Point3f>::const_iterator i=mCloud.begin();
      i!=mCloud.end();++i) {
      
      float x, y, z;
      Arm2Cam(*i, rot, &x, &y, &z);
      
      if(y<ROWS && y>0 && x<COLS && x>0){
	lDepth.at<float>(round(y), round(x)) = (float)z;
	cv::Point2f tmp(round(y), round(x));
	lPointVec.push_back(tmp);
	if(lCreateMask){
	  lMask.at<uchar>(round(y), round(x)) = 255;
	}
      }
      
  }
  
  lPoints = cv::Mat(lPointVec, true).reshape(1);
}

void PredictObject::Arm2Cam(const cv::Point3f pts, const Quat rot, float *x, float *y, float *z)
{
  cv::Point3f ptsInCam = pts-orig;
  cv::Vec3f v;
  cv::Vec3f vrot;
  v[0]=ptsInCam.x;
  v[1]=ptsInCam.y;
  v[2]=ptsInCam.z;
  vrot=rot.inverse()*v;

  *x = (vrot[0]*mFocal.x)/vrot[2]+mCentre.x;
  *y = (vrot[1]*mFocal.y)/vrot[2]+mCentre.y;
  
  *z = vrot[2];

}

bool PredictObject::ProjectPoint(const float x, const float y, 
				 const float z, const cv::Mat &lMask,
				 cv::flann::Index *lFLANNIdx, 
				 const cv::Mat &lPoints,
				 cv::Mat &lImg, cv::Mat &lOver, 
				 cv::Mat &lDepth,
				 std::pair<bool,float> &plaus)
{
  int n = lPoints.rows;
  cv::Mat m_indices(n, 1, CV_32S);
  m_indices.setTo(0);
  cv::Mat m_dists(n, 1, CV_32F);
  m_dists.setTo(0);
  
  float ptsDepth = z;
  bool overlap = false;
  

  if(y<ROWS && y>0 && x<COLS && x>0){
    float origDepth = lDepth.at<float>(round(y),round(x));
    if(origDepth==0 ){
      // look for nearest neighbour in original depth map
      cv::Mat lP(1,2,CV_32F);
      lP.at<float>(0,0) = round(y);
      lP.at<float>(0,1) = round(x);
      lFLANNIdx->knnSearch(lP, m_indices, m_dists, 1, 
			   cv::flann::SearchParams(32) );
      //float* dists_ptr = m_dists.ptr<float>(0);
      int* indices_ptr = m_indices.ptr<int>(0);
      int lNN = indices_ptr[0];
      int lx = lPoints.at<float>(lNN, 0);
      int ly = lPoints.at<float>(lNN, 1);
      origDepth = lDepth.at<float>(lx, ly);
      lDepth.at<float>(y, x) = origDepth;
    }
    
    if(lMask.at<uchar>(round(y),round(x))==0) /*&& 
						ptsDepth>origDepth)*/
       {
	   float diff = ptsDepth-origDepth;
	   ptsDepth>origDepth ? diff = ptsDepth-origDepth : diff=origDepth-ptsDepth;
	   /*
	     float old_diff = lImg.at<float>(round(y), round(x));
	     if(old_diff<diff)
	     lImg.at<float>(round(y), round(x)) = diff;
	   */
	   lImg.at<float>(round(y), round(x)) += diff;
	   plaus.first = false;
	   plaus.second = diff;
	   if(maxDiverge<diff)
	       maxDiverge=diff;
	   nDiverge++;
	   
       } else if (ptsDepth<origDepth){
	float diff = origDepth-ptsDepth;
	/*
	  float old_diff = lOver.at<float>(round(y), round(x));
	  if(old_diff<diff)
	  lOver.at<float>(round(y), round(x)) = diff;
	*/
	lOver.at<float>(round(y), round(x)) += diff;
	if(maxOverlap<diff)
	    maxOverlap=diff;
	nOverlap++;
	overlap = true;
    } else {
	float diff = ptsDepth-origDepth;
	plaus.first = true;
	plaus.second = diff;
	
	if(diff>maxHidden)
	    maxHidden = diff;
	nHidden++;
    }
  }

  return overlap;

}

bool PredictObject::ProjectPoint(const float x, const float y, 
				 const float z, const cv::Mat &lMask,
				 cv::flann::Index *lFLANNIdx, 
				 const cv::Mat &lPoints,
				 cv::Mat &lImg, cv::Mat &lOver, 
				 cv::Mat &lNImg, cv::Mat &lNOver, 
				 cv::Mat &lDepth,
				 std::pair<bool,float> &plaus)
{
  int n = lPoints.rows;
  cv::Mat m_indices(n, 1, CV_32S);
  m_indices.setTo(0);
  cv::Mat m_dists(n, 1, CV_32F);
  m_dists.setTo(0);
  
  float ptsDepth = z;
  bool overlap = false;
  

  if(y<ROWS && y>0 && x<COLS && x>0){
    float origDepth = lDepth.at<float>(round(y),round(x));
    if(origDepth==0 ){
      // look for nearest neighbour in original depth map
      cv::Mat lP(1,2,CV_32F);
      lP.at<float>(0,0) = round(y);
      lP.at<float>(0,1) = round(x);
      lFLANNIdx->knnSearch(lP, m_indices, m_dists, 1, 
			   cv::flann::SearchParams(32) );
      //float* dists_ptr = m_dists.ptr<float>(0);
      int* indices_ptr = m_indices.ptr<int>(0);
      int lNN = indices_ptr[0];
      int lx = lPoints.at<float>(lNN, 0);
      int ly = lPoints.at<float>(lNN, 1);
      origDepth = lDepth.at<float>(lx, ly);
      lDepth.at<float>(y, x) = origDepth;
    }
    
    if(lMask.at<uchar>(round(y),round(x))==0/* && ptsDepth>origDepth*/)
       {
      float diff = ptsDepth-origDepth;
      ptsDepth>origDepth ? diff = ptsDepth-origDepth : diff=origDepth-ptsDepth;
      
      lImg.at<float>(round(y), round(x)) += diff;
      lNImg.at<int>(round(y), round(x)) ++;
      plaus.first = false;
      plaus.second = diff;
      if(maxDiverge<diff)
	maxDiverge=diff;
      nDiverge++;

    } else if (ptsDepth<origDepth){
      float diff = origDepth-ptsDepth;
      

      lOver.at<float>(round(y), round(x)) += diff; 
      lNOver.at<int>(round(y), round(x)) ++;


      if(maxOverlap<diff)
	  maxOverlap=diff;
      nOverlap++;
      overlap = true;
    } else {
	float diff = ptsDepth-origDepth;
	plaus.first = true;
	plaus.second = diff;
	
	if(diff>maxHidden)
	    maxHidden = diff;
	nHidden++;
    }
  }
  

  return overlap;

}


void PredictObject::FillProjection(cv::Mat &lSrc, cv::Mat &lDst, bool sparse)
{
  if(!sparse){
    
    // fill in holes insode of outer contour
    vector<vector<cv::Point> > contours;
    vector<cv::Vec4i> hierarchy;
    cv::Scalar color( 255, 255, 255 );
    cv::Mat lSrcCopy(lSrc);
    
    cv::findContours( lSrcCopy, contours, hierarchy, 
		      CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE );
    int idx = 0;
    for( ; idx >= 0; idx = hierarchy[idx][0] ){
      drawContours( lDst, contours, idx, color, CV_FILLED, 8, hierarchy, 0);
    }

    cv::morphologyEx(lDst, lDst, cv::MORPH_OPEN, cv::Mat());
    cv::morphologyEx(lDst, lDst, cv::MORPH_CLOSE, cv::Mat());

  } else {
    
    // create convex contour and fill it
    std::vector<cv::Point> points;
    for (int y=0;y<lSrc.cols;y++){
      for (int x=0;x<lSrc.rows;x++){
	if(lSrc.at<uchar>(x,y)!=0){
	  cv::Point tmp;
	  tmp.x = y;
	  tmp.y = x;
	  points.push_back(tmp);
	}
      }
    }

    cv::Mat pointMat(points);
    std::vector<cv::Point> hull;
    convexHull(pointMat, hull);
    cv::Scalar color( 255, 255, 255 );
    std::vector<std::vector<cv::Point> > contours;
    contours.push_back(hull);
    lDst.setTo(0);
    drawContours(lDst, contours, -1, color, CV_FILLED);

  } 

}

void PredictObject::MirrorFromPlane( const std::vector<cv::Point3f> &lCloud,
				     const cv::Point3f lOrig,
				     const cv::Point3f lC,
				     const cv::Vec3f   lMirrAxis,
				     std::vector<cv::Point3f> &lOrigCloud, 
				     std::vector<cv::Point3f> &lMirCloud,
				     std::vector<cv::Point3f> &normals,
				     std::vector<bool> &original)
{
    
  cv::Point3f rOrig;
  Quat lRot;
    
  lRot.makeRotate(cv::Vec3f(0,1,lMirrAxis[2]),lMirrAxis);

  // mirroring the viewpoint only
  bool success = MirrorPoint(lOrig, rOrig, lRot, lC);
    
  if(!success){
    cout << "Failed to mirror viewpoint! " << endl;
    exit(-1);
  }
    
    
  // mirror whole point cloud
  for(std::vector<cv::Point3f>::const_iterator i=lCloud.begin();i!=lCloud.end();++i) {
    cv::Point3f pt;
    cv::Point3f pts(i->x, i->y, i->z);
    if(MirrorPoint(pts, pt, lRot, lC)) {

      lOrigCloud.push_back(pts);
      lMirCloud.push_back(pt);
    }
  }

  original.reserve(lMirCloud.size());
  original.resize(lMirCloud.size(), false);

  // compute normals
  cv::Mesh3D mesh_orig( lOrigCloud);
  mesh_orig.computeNormals(10.0);

  cv::Mesh3D mesh_mir( lMirCloud);
  mesh_mir.computeNormals(10.0);
    
  normals.reserve(mesh_mir.normals.size()+mesh_orig.normals.size());
  normals.resize(mesh_mir.normals.size()+mesh_orig.normals.size());
    
  for(unsigned int i=0; i < mesh_mir.normals.size();i++){
    flipNormalTowardsViewpoint(mesh_mir.normals[i],lMirCloud[i],rOrig);
    normals.at(i)=mesh_mir.normals.at(i);
  }

  for(unsigned int i=0; i < mesh_orig.normals.size();i++){
    flipNormalTowardsViewpoint(mesh_orig.normals[i],lOrigCloud[i],lOrig);
    normals.at(i+mesh_mir.normals.size())=mesh_orig.normals.at(i);
  }

  for(unsigned int i=0; i< lOrigCloud.size(); i++){
    lMirCloud.push_back(lOrigCloud.at(i));
    original.push_back(true);
  }
}



void PredictObject::MirrorCloud( const cv::Mat &lPoints, 
				 cv::flann::Index &lFLANNIdx,
				 const cv::Vec3f avg3D,
				 std::vector<cv::Point3f> &lab_cloud, 
				 std::vector<cv::Point3f> &lMirCloud,
				 std::vector<cv::Point3f> &normals,
				 std::vector<float> &plausibilities,
				 std::vector<bool> &original)
{
  // centre of mass of table projection
  cv::Vec3f avgV;
  // viewing Direction
  cv::Vec3f viewDir;
  // initial mirroring axis
  cv::Vec3f MirrAxis; 
  // axis perpendicular to mirroring axis
  cv::Vec3f PerpAxis;
  // quaternion to mirror point cloud around axis
  Quat rot;
  // determines wether x or y coordinate is mirrored
  int idxToMirror;

  // Get above values based on principal components of table projection 
  // in relation to viewing direction
  GetFixedSurfaceParameters(avgV, viewDir, MirrAxis, PerpAxis, rot, idxToMirror);

  // storage for mirrored and original clouds
  vector<vector<cv::Point3f> > lMirroredClouds;
  vector<vector<cv::Point3f> > lOrigClouds;
  // storage for votes for each cloud
  vector<Vote> lVotes;
  // rotated camera origins necessary for creating the mesh
  vector<cv::Point3f> lRotOrigs;
  vector<float> lStepSizes;
  vector<vector<std::pair<bool,float> > > lPlausibilities;

  float shift3D, stepsize;

  cv::Mat rgbProj;
  rgbProj.create(mProj.size(),CV_8UC3);
  cv::cvtColor(mProj,rgbProj,CV_GRAY2RGB);

  MirrorShift(lMirroredClouds, lOrigClouds, 
	      lVotes, lRotOrigs, lStepSizes,
	      lPlausibilities,
	      lPoints, lFLANNIdx, avg3D, avgV, viewDir, 
	      MirrAxis, PerpAxis, rot, idxToMirror,
	      shift3D, stepsize, &rgbProj);
  CalcVotes(lVotes, shift3D, 0.5);
  int bestIDX = SelectBestCloud(lVotes);  

  
  // mark best centroid
  float shift = stepsize/2.0f * bestIDX;
  if((-viewDir).dot(PerpAxis)*shift<0)
    shift = -shift;
  
  cv::Vec3f ShiftVec = PerpAxis*shift;
  cv::Point3f tmpMean;  
  tmpMean.x = avg->data.fl[0]+minX+ShiftVec[0];
  tmpMean.y = avg->data.fl[1]+minY+ShiftVec[1];
  tmpMean.z = maxZ;

  mPlanePoint = tmpMean;
  mPlaneNormal = PerpAxis;

  CvPoint avgNew;
  avgNew.x = tmpMean.y-minY;
  avgNew.y = tmpMean.x-minX;

  cv::circle(rgbProj, avgNew, 5, CV_RGB( 255, 0, 0 ));
  ///////////////////////////////////////////////////  
  
  std::cout << "bestCloud " << bestIDX << std::endl;
  lab_cloud = lOrigClouds[bestIDX];
  cv::Vec3f rotOrig = lRotOrigs[bestIDX];


  cv::Mesh3D mesh_orig( lab_cloud);
  mesh_orig.computeNormals(10.0);

  lMirCloud = lMirroredClouds[bestIDX];
  plausibilities.reserve(lPlausibilities[bestIDX].size()+lab_cloud.size());
  for(std::vector<std::pair<bool,float> >::iterator 
	i=lPlausibilities[bestIDX].begin();
      i!=lPlausibilities[bestIDX].end();++i)
    plausibilities.push_back(i->second);
  original.reserve(lMirCloud.size());
  original.resize(lMirCloud.size(), false);

  cv::Mesh3D mesh_mir( lMirCloud);
  mesh_mir.computeNormals(10.0);

  normals.reserve(mesh_mir.normals.size()+mesh_orig.normals.size());
  normals.resize(mesh_mir.normals.size()+mesh_orig.normals.size());
    
  for(unsigned int i=0; i < mesh_mir.normals.size();i++){
    flipNormalTowardsViewpoint(mesh_mir.normals[i],lMirCloud[i],rotOrig);
    normals.at(i)=mesh_mir.normals.at(i);
  }

  for(unsigned int i=0; i < mesh_orig.normals.size();i++){
      flipNormalTowardsViewpoint(mesh_orig.normals[i],lab_cloud[i],orig);
      normals.at(i+mesh_mir.normals.size())=mesh_orig.normals.at(i);
  }

  for(unsigned int i=0; i< lab_cloud.size(); i++){
    lMirCloud.push_back(lab_cloud.at(i));
    plausibilities.push_back(orig_plaus);
    original.push_back(true);
  }
  
  assert(lMirCloud.size()==plausibilities.size());
  assert(lMirCloud.size()==original.size());
}

void PredictObject::MirrorShift(std::vector<std::vector<cv::Point3f> > &lMirroredClouds,
				std::vector<std::vector<cv::Point3f> > &lOrigClouds,
				std::vector<Vote> &lVotes,
				std::vector<cv::Point3f> &lRotOrigs,
				std::vector<float> &lStepSizes,
				std::vector<std::vector<std::pair<bool,float> > > &lPlausibilities,
				const cv::Mat &lPoints, 
				cv::flann::Index &lFLANNIdx,
				const cv::Vec3f avg3D,
				const cv::Vec3f avgV, 
				const cv::Vec3f viewDir, 
				const cv::Vec3f MirrAxis, 
				const cv::Vec3f PerpAxis, 
				const Quat rot, const int idxToMirror,
				float &shift3D, float &stepsize,
				cv::Mat *rgbProj)
{
  static int calls = 0;

  // running variable for determining shift
  int offset;
  // Dependent on size of point cloud and number of hypotheses considered 
  // this setpsize will be computed 
  stepsize = 0;
  // computes the extend of the mirrored point cloud from min/maxIdxToMirror
  shift3D = 0;
  float minIdxToMirror = 0;
  float maxIdxToMirror = 0;

  // mirrored Camera origin across axis
  cv::Point3f rotOrig;
  
  std::vector<cv::Point3f> lab_cloud;
  std::vector<cv::Point3f> lMirCloud;
  std::vector<std::pair<bool,float> > lPlaus;
  cv::Mat testProj;
  cv::Mat dummy;
  
  for(offset=0; offset<HYPOS; offset++) {
    
    //    testProj = cv::Mat::zeros(mProj.size(),CV_8UC3);

    std::cout << "Checking offset "<< offset << std::endl;

    lab_cloud.clear();
    lMirCloud.clear();
    lPlaus.clear();
    
    // check all the input parameters
    /*
    std::cout << "Rot " << rot._v[0] << " " << rot._v[1] << " "  
	      << rot._v[2] << " "  << rot._v[3] << " "<< std::endl;
    std::cout << "Centre " << avgV[0] << " " << avgV[1] << " " << avgV[2] << std::endl;
    std::cout << "IdxToMirror " << idxToMirror << std::endl;
    std::cout << "offset " << offset << std::endl;
    std::cout << "stepsize " << stepsize << std::endl;
    std::cout << "tmp offset " <<  stepsize*offset << std::endl;
    */

    // mirroring the viewpoint only
    bool success = MirrorPoint(orig, rotOrig, rot, avgV, idxToMirror,
			       minIdxToMirror,  maxIdxToMirror, offset, stepsize, dummy);
    minIdxToMirror = 0;
    maxIdxToMirror = 0;
    
    if(!success){
      cout << "Failed to mirror viewpoint! " << endl;
      exit(-1);
    }
    
    
    // mirror whole point cloud
    for(std::vector<cv::Point3f>::iterator i=mCloud.begin();i!=mCloud.end();++i) {
      cv::Point3f pt;
      cv::Point3f pts(i->x, i->y, i->z);
      if(MirrorPoint(pts, pt, rot, avgV, idxToMirror, 
		     minIdxToMirror, maxIdxToMirror, offset, stepsize,
		     testProj)) {

	lab_cloud.push_back(pts);
	lMirCloud.push_back(pt);
      }
    }
    
    
    if(offset==0){
      // get extend of mirrored point cloud along perpendicular axis
      stepsize = minIdxToMirror/HYPOS;
      if(minIdxToMirror<maxIdxToMirror){
	stepsize = std::abs(stepsize);
	lStepSizes.push_back(stepsize);
	shift3D = maxIdxToMirror - minIdxToMirror;
      } else {
	stepsize = -std::abs(stepsize);
	lStepSizes.push_back(stepsize);
	shift3D = minIdxToMirror - maxIdxToMirror;
      }


    }
    
    // shift of old centroid to mark in debug image
    float shift = stepsize/2.0f * offset;
  
    if((-viewDir).dot(PerpAxis)*shift<0){
      shift = -shift;
    }
    
    cv::Vec3f ShiftVec = PerpAxis*shift;
    cv::Point3f tmpMean;
    tmpMean.x = avg->data.fl[0]+minX+ShiftVec[0];
    tmpMean.y = avg->data.fl[1]+minY+ShiftVec[1];
    tmpMean.z = maxZ;

    CvPoint avgNew;
    avgNew.x = tmpMean.y-minY;
    avgNew.y = tmpMean.x-minX;
    
    CvPoint avgOld;
    avgOld.x = avg->data.fl[1];
    avgOld.y = avg->data.fl[0];

    if(rgbProj!=NULL){
      cv::line( *rgbProj, avgOld, avgNew, CV_RGB( 0, 255, 0 ));
      
     }
    //////////////////////////////////////////////
    
    cv::Mat lImg;
    cv::Mat lOver;

    ProjectOnImage(lMirCloud, mMask, avg3D, &lFLANNIdx, 
		   lPoints, lImg, lOver, mDepthLU, lPlaus);
    
    Vote lVote = GetVote(lImg, lOver);
    
    Divergence2Plausability(lPlaus);
    
    lMirroredClouds.push_back(lMirCloud);
    lOrigClouds.push_back(lab_cloud);
    lVotes.push_back(lVote);
    lRotOrigs.push_back(rotOrig);
    lPlausibilities.push_back(lPlaus);

   
  }

  calls++;
}

void PredictObject::MirrorCloudSearchOrient( const cv::Mat &lPoints, 
					     cv::flann::Index &lFLANNIdx,
					     const cv::Vec3f avg3D,
					     std::vector<cv::Point3f> &lab_cloud, 
					     std::vector<cv::Point3f> &lMirCloud,
					     std::vector<cv::Point3f> &normals,
					     std::vector<float> &plausibilities,
					     std::vector<bool> &original)
{
  // centre of mass of table projection
  cv::Vec3f avgV;
  // viewing Direction
  cv::Vec3f viewDir;
  // initial mirroring axis
  cv::Vec3f MirrAxis; 
  // axis perpendicular to mirroring axis
  cv::Vec3f PerpAxis;
  // quaternion to mirror point cloud around axis
  Quat rot;
  // determines wether x or y coordinate is mirrored
  int idxToMirror;

  // Get above values based on principal components of table projection 
  // in relation to viewing direction
  GetFixedSurfaceParameters(avgV, viewDir, MirrAxis, PerpAxis, rot, idxToMirror);

  // storage for mirrored and original clouds
  vector<vector<cv::Point3f> > lMirroredClouds;
  vector<vector<cv::Point3f> > lOrigClouds;
  map<pair<int,int>, Vote> lVotes;
  vector<float> lStepSizes;
  vector<vector<std::pair<bool,float> > > lPlausibilities;

  // rotated camera origins necessary for creating the mesh
  vector<cv::Point3f> lRotOrigs;

  float shift3D, stepsize;

  cv::Mat rgbProj;
  rgbProj.create(mProj.size(),CV_8UC3);
  cv::cvtColor(mProj,rgbProj,CV_GRAY2RGB);

  float angleSteps = DegToRad(2.0f*(float)MINMAXANGLE/(float)ANGLEHYPOS);
  //std::cout << "Angle Steps " << angleSteps << std::endl;
  int angleOffset=0;



  for( float alpha=DegToRad(-(float)MINMAXANGLE); 
       alpha<=DegToRad((float)MINMAXANGLE); 
       alpha+= angleSteps ) {
        
    std::cout << "Checking Angle "<< RadToDeg(alpha) << " Degrees." << std::endl;
    
    // storage for votes for each cloud
    vector<Vote> lVotesShiftOnly;

    Quat subRot;
    subRot.makeRotate( alpha, cv::Vec3f(0,0,1));
    cv::Vec3f newMirrAxis = subRot.inverse() * MirrAxis;
    cv::Vec3f newPerpAxis = subRot.inverse() * PerpAxis;
    
    float dotProdx = cv::Vec3f(1,0,newMirrAxis[2]).dot( newMirrAxis);
    float dotPrody = cv::Vec3f(0,1,newMirrAxis[2]).dot( newMirrAxis);
    Quat rot;
    
    int idxToMirror = 0;
    if( dotProdx > dotPrody){
      rot.makeRotate(cv::Vec3f(1,0,newMirrAxis[2]),newMirrAxis);
      idxToMirror = 1;
    } else {
      
      rot.makeRotate(cv::Vec3f(0,1,newMirrAxis[2]),newMirrAxis);
    }
    
    MirrorShift(lMirroredClouds, 
		lOrigClouds, lVotesShiftOnly, lRotOrigs, lStepSizes,
		lPlausibilities,
		lPoints, lFLANNIdx, avg3D, avgV, viewDir, 
		newMirrAxis, newPerpAxis, rot, idxToMirror,
		shift3D, stepsize, &rgbProj);

    for(int offset=0; offset<HYPOS; offset++) 
      lVotes.insert(pair< pair<int, int>, Vote >(pair<int, int>(angleOffset, offset), lVotesShiftOnly[offset] ));
    
    angleOffset++;
  }
  
  std::cout << "MaxDiverge " << maxDiverge << std::endl;
  std::cout << "MaxOverlap " << maxOverlap << std::endl;
  //CalcVotes(lVotes, 0.4f);
  CalcVotes(lVotes, 0.5f);

  pair<int, int> bestPairIDX = SelectBestCloud(lVotes);  

  std::cout << "Best offset and angle " << bestPairIDX.second << " " 
	    << bestPairIDX.first << std::endl;

  int bestIDX = bestPairIDX.second;
  int bestAngleIDX = bestPairIDX.first;


  float bestAngle = DegToRad(-(float)MINMAXANGLE) + bestAngleIDX*angleSteps;
  // shift of old centroid
  float lStepsize = lStepSizes[bestAngleIDX];
  float shift = lStepsize/2.0f * bestIDX;
  std::cout << "Bestidx " << bestIDX <<  " SHIFT " << shift << std::endl;
  std::cout << "Best Stepsize " << lStepsize << std::endl;

  std::cout << "Best avg " <<  avg->data.fl[0] << " " << avg->data.fl[1]<< std::endl;
  
  Quat subRot;
  subRot.makeRotate( bestAngle, cv::Vec3f(0,0,1));
  cv::Vec3f newPerpAxis = subRot.inverse() * PerpAxis;
  //cv::Vec3f newPerpAxis = subRot * PerpAxis;

  

  float scale = std::sqrt((shift*shift)/(newPerpAxis[0]*newPerpAxis[0] + newPerpAxis[1]*newPerpAxis[1]));
  std::cout << " Scale " << scale << std::endl;
  
  if((-viewDir).dot(newPerpAxis)*shift<0)
    shift = -shift;
  
  cv::Vec3f ShiftVec = newPerpAxis*shift;
  cv::Point3f tmpMean;
  tmpMean.x = avg->data.fl[0]+minX+ShiftVec[0];
  tmpMean.y = avg->data.fl[1]+minY+ShiftVec[1];
  tmpMean.z = maxZ;

  mPlanePoint = tmpMean;
  mPlaneNormal = newPerpAxis;

  CvPoint avgNew;
  avgNew.x = tmpMean.y-minY;
  avgNew.y = tmpMean.x-minX;

  CvPoint avgOld;
  avgOld.x = avg->data.fl[1];
  avgOld.y = avg->data.fl[0];

  CvPoint tip;
  tip.x = ShiftVec[1];//newPerpAxis[1]*5.0;
  tip.y = ShiftVec[0];//newPerpAxis[0]*5.0;
  
  //  cv::line( rgbProj,pt0_S, pt_S, CV_RGB( 0, 255, 0 ));
  cv::line(rgbProj,avgOld,tip,CV_RGB( 0, 0, 255 ));
  //  cv::line(rgbProj, avgNew, tip, CV_RGB( 0, 0, 255 ));
  cv::circle(rgbProj, avgNew, 5, CV_RGB( 255, 0, 0 ));

  int bestCloud = HYPOS*bestAngleIDX+bestIDX;
  std::cout << "bestCloud " << bestCloud << std::endl;
  lab_cloud = lOrigClouds[bestCloud];
  lMirCloud = lMirroredClouds[bestCloud];

  plausibilities.reserve(lPlausibilities[bestCloud].size()
			 +lab_cloud.size());
  for(std::vector<std::pair<bool,float> >::iterator 
	i=lPlausibilities[bestCloud].begin();
      i!=lPlausibilities[bestCloud].end();++i)
    plausibilities.push_back(i->second);
  original.reserve(lMirCloud.size());
  original.resize(lMirCloud.size(), false);

  cv::Vec3f rotOrig = lRotOrigs[bestCloud];

  //std::vector<cv::Point3f> lab_cloud_cp;
  //ConvertPC(lab_cloud, lab_cloud_cp);
  cv::Mesh3D mesh_orig( lab_cloud);
  mesh_orig.computeNormals(10.0);

  //  std::vector<cv::Point3f> lMirCloud_cp;
  //  ConvertPC(lMirCloud, lMirCloud_cp);
  cv::Mesh3D mesh_mir( lMirCloud);
  mesh_mir.computeNormals(10.0);
  
  normals.reserve(mesh_mir.normals.size()+mesh_orig.normals.size());
  normals.resize(mesh_mir.normals.size()+mesh_orig.normals.size());

  for(unsigned int i=0; i < mesh_mir.normals.size();i++){
      flipNormalTowardsViewpoint(mesh_mir.normals[i],lMirCloud[i],rotOrig);
      normals.at(i)=mesh_mir.normals.at(i);
  }

  for(unsigned int i=0; i < mesh_orig.normals.size();i++){
      flipNormalTowardsViewpoint(mesh_orig.normals[i],lab_cloud[i],orig);
      normals.at(i+mesh_mir.normals.size())=mesh_orig.normals.at(i);
  }
  
  char name[512];
  sprintf(name, "onlyMirrored.crd");
  DumpMirroredCloud(name, lMirCloud);

  // calc Normals on full cloud 
  for(unsigned int i=0; i< lab_cloud.size(); i++){
    lMirCloud.push_back(lab_cloud[i]);
    plausibilities.push_back(orig_plaus);
    original.push_back(true);
  }

  std::cout << "Size of lMirCLoud and plausibilities " <<
    lMirCloud.size() << " " << plausibilities.size() << std::endl;

  assert(lMirCloud.size()==plausibilities.size());
  assert(lMirCloud.size()==original.size());
}

void PredictObject::GetFixedSurfaceParameters(cv::Vec3f &avgV,
					      cv::Vec3f &viewDir,
					      cv::Vec3f &lMirrAxis, 
					      cv::Vec3f &lPerpAxis,
					      Quat &lRot, 
					      int &idxToMirror)
{
  float zavg = 0.0;
  avgV = cv::Vec3f(avg->data.fl[0]+minX, avg->data.fl[1]+minY, zavg);
  cv::Vec3f eigV1A( eigenVectors->data.fl[0], 
		    eigenVectors->data.fl[1],
		    zavg);
  cv::Vec3f eigV2A( eigenVectors->data.fl[2], 
		    eigenVectors->data.fl[3],
		    zavg);
  
  // check which eigenvector is more perpendicular to viewing direction
  if(mHaveTrans){
    // use z-Axis of Camera if available
    CV_Assert(mTrans.rows == 3 && mTrans.cols == 4);
    cv::Mat lRotMat = mTrans.colRange(0,mTrans.cols-1);
    
    cv::Mat    v(3,1,CV_32F);
    v.at<float>(0,0)=orig.x;
    v.at<float>(1,0)=orig.y;
    v.at<float>(2,0)=orig.z;

    cv::Mat    tmp(3,1,CV_32F);
    tmp.setTo(0.0f);

    cv::Mat vrot(3,1,CV_32F);


    gemm(lRotMat.t(), v, 1.0f, tmp, 1.0f,vrot);
    viewDir = cv::Vec3f(vrot.at<float>(0,0), vrot.at<float>(1,0), zavg);
      
  } else {
    // use vector between camera origin and centre of mass of point cloud
    viewDir = cv::Vec3f(orig.x, orig.y, zavg)-avgV;
  }

  normalize(viewDir);
  
  lMirrAxis = eigV1A;
  lPerpAxis = eigV2A;
  
 
  // determine which is the shortest rotation direction 
  // (alignment of mirroring axis with x or with y-Axis)
  float dotProdx = cv::Vec3f(1,0,lMirrAxis[2]).dot( lMirrAxis);
  float dotPrody = cv::Vec3f(0,1,lMirrAxis[2]).dot( lMirrAxis);

  idxToMirror = 0;
  if( dotProdx > dotPrody){
    lRot.makeRotate(cv::Vec3f(1,0,lMirrAxis[2]),lMirrAxis);
    idxToMirror = 1;
  } else {
    lRot.makeRotate(cv::Vec3f(0,1,lMirrAxis[2]),lMirrAxis);
  }

}

void PredictObject::Divergence2Plausability(std::vector<std::pair<bool,float> > &lPlaus)
 {
   // Convert divergence to plausibility by normalising it and 
   // inverting it with respect to [0...1] range
   for(std::vector<std::pair<bool,float> >::iterator i=lPlaus.begin();
       i!=lPlaus.end();++i)
     {
       if(i->first){
	 // point was in occluded area
	   // This should probbaly be changed to 
	   // i->second = MapRanges( maxHidden, 0,
           // orig_plaus,0.5,
           // i->second);
	   // to say that point farer away from original points fall off 
	   // in probability
	   // THE SAME WITH DIVERGENCE
	 i->second = MapRanges( maxHidden, 0,
				0.5, orig_plaus,
				i->second);
       } else {
	 // point appears in visible space
	 i->second = MapRanges( maxDiverge, 0,
				0, orig_plaus, 
				i->second);
       }
     }
 }

PredictObject::Vote PredictObject::GetVote( const cv::Mat &lImg, 
					    const cv::Mat &lOver)
{
  Vote lVote;
  int nNonZeros=-1;
  cv::Size matSize;
  
  double minVal, maxVal;
  cv::minMaxLoc(lImg, &minVal, &maxVal);
  if(maxDiverge<maxVal)
    maxDiverge = maxVal;
  
#if MEDIAN==1
  if(maxVal>0){
    cv::Mat lProd = lImg.reshape(0,1);
    cv::Mat lProdSort = cv::Mat::zeros(lProd.rows, lProd.cols, CV_32F);
    cv::sort(lProd,lProdSort,CV_SORT_EVERY_ROW + CV_SORT_ASCENDING);
    nNonZeros = countNonZero(lProdSort);
    
    matSize = lProdSort.size();
    int median = floor(nNonZeros/2.0f);
    
    /*
    std::cout << "......................................" << std::endl;
    std::cout << "nNonZeros " << nNonZeros << std::endl;
    std::cout << "sizeofVector " << matSize.width*matSize.height << std::endl;
    std::cout << "median in nonZeros  " << median << std::endl;
    std::cout << "offset from zeros " << matSize.width*matSize.height-nNonZeros << std::endl;
    std::cout << "Median " << matSize.width*matSize.height-nNonZeros+median << std::endl;
    std::cout << "Diverge val " << lProdSort.at<float>( 0, matSize.width*matSize.height-nNonZeros+median) << std::endl;
    */
    
    lVote.divergeMedian = lProdSort.at<float>( 0, matSize.width*matSize.height-nNonZeros+median);
    
  } else {
    
    lVote.divergeMedian = 0;
    
  }
  
#else 
  
  nNonZeros = cv::countNonZero(lImg);

  if(nNonZeros>0){
    cv::Scalar lDivergeVal  = cv::sum(lImg);
    lVote.divergeVal = lDivergeVal[0];
   } else {
    lVote.divergeVal = 0;
  }
  
#endif
  
  if(nNonZeros==-1)
    lVote.nPix = cv::countNonZero(lImg);
  else 
    lVote.nPix = nNonZeros;
  
  
  //lVote.nPix = nDiverge;

  
  cv::minMaxLoc(lOver, &minVal, &maxVal);
  if(maxOverlap<maxVal)
    maxOverlap = maxVal;
  

  nNonZeros=-1;

#if MEDIAN==1
  if(maxDiverge>0){
    cv::Mat lOverTmp=lOver.reshape(0,1);
    cv::sort(lOverTmp,lOverTmp,CV_SORT_EVERY_ROW + CV_SORT_ASCENDING);
    nNonZeros = cv::countNonZero(lOverTmp);
    matSize = lOverTmp.size();
    int median = floor(nNonZeros/2.0f);
    
    /*
    std::cout << "......................................" << std::endl;
    std::cout << "............lOverTmp.................." << std::endl;
    std::cout << "nNonZeros " << nNonZeros << std::endl;
    std::cout << "sizeofVector " << matSize.width*matSize.height << std::endl;
    std::cout << "median in nonZeros  " << median << std::endl;
    std::cout << "offset from zeros " << matSize.width*matSize.height-nNonZeros << std::endl;
    std::cout << "Median " << matSize.width*matSize.height-nNonZeros+median << std::endl;
    std::cout << "overlap val " << (float)lOverTmp.at<float>( 0, matSize.width*matSize.height-nNonZeros+median) << std::endl;
    */
    
    lVote.overlapMedian = lOverTmp.at<float>( 0, matSize.width*matSize.height-nNonZeros+median);
    
  } else {
    
    lVote.overlapMedian = 0;
    
  }
  
#else 
  
  cv::Scalar lOverlapVal = cv::sum(lOver);
  lVote.overlapVal = lOverlapVal[0];
  
#endif

  
  if(nNonZeros==-1)
    lVote.nPoints = cv::countNonZero(lOver);
  else 
    lVote.nPoints = nNonZeros;
  

  //lVote.nPoints = nOverlap;

  return lVote;
  
}

void PredictObject::CalcVotes( std::vector<Vote> &lVotes, 
			       float maxShift3D, 
			       float lWDiverge)
{
  for(vector<Vote>::iterator i=lVotes.begin();i!=lVotes.end();++i) {
    
    float lVote;
    
    float wDiverge = lWDiverge;
    float wOverlap = 1-wDiverge;

    float lMaxDiverge;
    float lMaxOverlap;

    if(maxShift3D == 0){
      lMaxDiverge = maxDiverge;
      lMaxOverlap = maxOverlap;
    } else {
      lMaxDiverge = maxShift3D;
      lMaxOverlap = maxShift3D;
    }

#if MEDIAN==1

    float divergeVal = i->divergeMedian/lMaxDiverge;
    float overlapVal = i->overlapMedian/lMaxOverlap;

    lVote = wDiverge*divergeVal + wOverlap*overlapVal;

    
#else

    float divergeVal = i->divergeVal/lMaxDiverge;
    float overlapVal = i->overlapVal/lMaxOverlap;


    
    if(i->nPix > 0)
      lVote = wDiverge*(divergeVal/i->nPix);
    else 
      lVote = 0;
    
    if(i->nPoints > 0)
      lVote += wOverlap*(overlapVal/i->nPoints);
    else 
      lVote += 0;

#endif
    
    i->val = lVote;

  }
}

void PredictObject::CalcVotes( std::map<pair<int,int>, Vote> &lVotes, 
			       float lWDiverge)
{
  float wDiverge = lWDiverge;
  float wOverlap = 1.0f-wDiverge;

  map<pair<int, int>, Vote >::iterator end = lVotes.end(); 
  for(map<pair<int, int>, Vote >::iterator it = lVotes.begin(); it != end; ++it){
  
    float lVote;
    
#if MEDIAN==1

    float divergeVal = it->second.divergeMedian/maxDiverge;
    float overlapVal = it->second.overlapMedian/maxOverlap;

    lVote = wDiverge*divergeVal + wOverlap*overlapVal;

    
#else

    float divergeVal = it->second.divergeVal/maxDiverge;
    float overlapVal = it->second.overlapVal/maxOverlap;


    
    if(it->second.nPix > 0)
      lVote = wDiverge*(divergeVal/it->second.nPix);
    else 
      lVote = 0;
    
    if(it->second.nPoints > 0)
      lVote += wOverlap*(overlapVal/it->second.nPoints);
    else 
      lVote += 0;

#endif
    
    it->second.val = lVote;

  }
}

int PredictObject::SelectBestCloud(const std::vector<Vote> lVotes)
{
  
  float oldTmp=1000;
  int best = -1;
#if GLOBALMIN == 1
  std::cout << "Searching for global best cloud " << std::endl;
  for(unsigned int i=0; i<lVotes.size();++i){
    std::cout << lVotes[i].val << std::endl;
    if(lVotes[i].val<oldTmp){
      oldTmp=lVotes[i].val;
      best=i;
    }
  }

#elif GLOBALMIN == 0

  std::cout << "Ordered Votes " << std::endl;
  for(int i=lVotes.size(); i>0; i--){
    float tmp = lVotes[i-1].val;
    if(oldTmp<tmp){
      break;
    }
    best = i-1;
    oldTmp=tmp;
    
    std::cout << lVotes[i-1].val << std::endl;
  }
  
#endif

  bestVote = oldTmp;
  return best;
}

pair<int,int> PredictObject::SelectBestCloud(const map<pair<int, int>, Vote > lVotes)
{
  
#if GLOBALMIN == 1
  float minVote=100;
  pair<int,int> best;
  best.first = -1;
  best.second= -1;
  
  map<pair<int, int>, Vote >::const_iterator end = lVotes.end(); 
  for(map<pair<int, int>, Vote >::const_iterator it = lVotes.begin(); it != end; ++it){
    
    std::cout << "Votes " <<  it->second.val << std::endl;

    if(minVote > it->second.val){
      minVote=it->second.val;
      best=it->first;

      std::cout << "new minVote " << minVote << std::endl;
      std::cout << best.first << " " << best.second << std::endl;

    } 
  }

  bestVote = minVote;

#elif GLOBALMIN == 0

  std::vector<int> best4Angles;
  map<pair<int, int>, Vote >::const_iterator it;
  pair<int,int> best;
  best.first = -1;
  best.second= -1;

  // first go over angles and search for best there
  std::cout << "Search over angles " << std::endl;
  for(int i=0; i<ANGLEHYPOS;i++){
    float oldTmp=10;
    int best = -1;
    pair<int, int> tmpPair;
    tmpPair.first=i;
    for(int j=HYPOS; j>0;j--){
      tmpPair.second=j-1;
      it = lVotes.find(tmpPair);
      float tmp = it->second.val;
      std::cout << "i,j,v " << i << " " << j-1 << " " << tmp << std::endl;
      if(oldTmp<tmp){
	break;
      }
      best = j-1;
      oldTmp=tmp;
    }
    best4Angles.push_back(best);
  }

  std::cout << "Search over min in Angles  " << std::endl;
  float minVote = 10.0f;
  // first search over minima per angle
  for(int i=0; i<ANGLEHYPOS;i++){
    pair<int, int> tmpPair;
    tmpPair.first=i;
    tmpPair.second=best4Angles[i];
    it = lVotes.find(tmpPair);

    std::cout << "i,j,v " << i << " " << best4Angles[i] << " " << it->second.val << std::endl;

    if(minVote>it->second.val){
      minVote=it->second.val;
      best.first=i;
      best.second=best4Angles[i];
    }      
  }

  bestVote = minVote;

#endif
  
  return best;
}

bool PredictObject::MirrorPoint(const cv::Point3f pts, 
				cv::Point3f &mirpts,
				const Quat lRot,
				const cv::Point3f lC)
{
    cv::Vec3f v;
    cv::Vec3f tmp;
    cv::Vec3f vrot;
    v[0]=pts.x;
    v[1]=pts.y;
    v[2]=pts.z;
    v[0]-=lC.x;
    v[1]-=lC.y;
    
    tmp=lRot.inverse()*v;
    tmp[1]= -tmp[1];
    
    vrot=lRot*tmp;
    vrot[0]+=lC.x;
    vrot[1]+=lC.y;
    mirpts.x=vrot[0];
    mirpts.y=vrot[1];
    mirpts.z=vrot[2];
    
    
    return true;

}

bool PredictObject::MirrorPoint(const cv::Point3f pts, cv::Point3f &mirpts,
				const Quat rot, const cv::Vec3f avgV, const int idxToMirror, 
				float &minIdxToMirror, float &maxIdxToMirror, 
				const int offset, const float stepsize,
				cv::Mat &lProj)
{
  float tmpOffset;
  
  cv::Vec3f vv;
  vv[0] = orig.x;
  vv[1] = orig.y;
  vv[2] = orig.z;
  vv[0]-= avgV[0];
  vv[1]-= avgV[1];
  cv::Vec3f rotvv=rot.inverse()*vv;


  cv::Vec3f v;
  cv::Vec3f tmp;
  cv::Vec3f vrot;
  v[0]=pts.x;
  v[1]=pts.y;
  v[2]=pts.z;
  v[0]-=avgV[0];
  v[1]-=avgV[1];

  tmp=rot.inverse()*v;
  
  if(!lProj.empty()){
    int r = tmp[0]+lProj.rows/2.0f;
    int c = tmp[1]+lProj.cols/2.0f;
    if(!(r>lProj.rows || r<0 || c>lProj.cols || c<0)){
      lProj.ptr<uchar>(r)[3*c+0] = 0;
      lProj.ptr<uchar>(r)[3*c+1] = 255;
      lProj.ptr<uchar>(r)[3*c+2] = 0;
    }
    
    CvPoint zero_p;
    zero_p.x = lProj.rows/2.0f;
    zero_p.y = lProj.cols/2.0f;
    cv::circle(lProj, zero_p, 10, CV_RGB( 255, 0, 0 ));
  }


  if(offset==0){
    //    std::cout << "Rotated Viewpoint " << rotvv[0] << " " << rotvv[1] << std::endl;
    if( std::abs(rotvv[idxToMirror]-tmp[idxToMirror]) > std::abs(rotvv[idxToMirror]-minIdxToMirror)){
      minIdxToMirror = tmp[idxToMirror];
    }

    if( std::abs(rotvv[idxToMirror]-tmp[idxToMirror]) < std::abs(rotvv[idxToMirror]-maxIdxToMirror)){
      maxIdxToMirror = tmp[idxToMirror];
    }
  }
  

  CvPoint zero_p;
  zero_p.x = 0+lProj.rows/2.0f;
  zero_p.y = 0+lProj.cols/2.0f;

  tmpOffset = stepsize*offset;
  tmp[idxToMirror]= (-(tmp[idxToMirror]+tmpOffset));
   
  if(!lProj.empty()){
    int r = tmp[0]+lProj.rows/2.0f;
    int c = tmp[1]+lProj.cols/2.0f;
    if(!(r>lProj.rows || r<0 || c>lProj.cols || c<0)){
      lProj.ptr<uchar>(r)[3*c+0] = 255;
      lProj.ptr<uchar>(r)[3*c+1] = 0;
      lProj.ptr<uchar>(r)[3*c+2] = 0;
    }

    
    if(idxToMirror==0)
      zero_p.y -= tmpOffset;
    else 
      zero_p.x -= tmpOffset;
    cv::circle(lProj, zero_p, 10, CV_RGB( 255, 0, 0 ));
  }
 
  cv::Vec3f zero_cv;
  zero_cv[0]= 0.0;
  zero_cv[1]= 0.0;
  zero_cv[2]= pts.z;
  
  if(idxToMirror==0)
    zero_cv[0] -= tmpOffset;
  else 
    zero_cv[1] -= tmpOffset;

  zero_cv = rot*zero_cv;

  vrot=rot*tmp;
  vrot[0]+=avgV[0];
  vrot[1]+=avgV[1];
  mirpts.x=vrot[0];
  mirpts.y=vrot[1];
  mirpts.z=vrot[2]; 


  //  std::cout << "center " << zero_cv[0] +avgV[0] << " " << zero_cv[1] +avgV[1] << std::endl;

  return true;
}

bool PredictObject::DemirrorPoint(cv::Point3f &pts,
				  const Quat rot, const cv::Vec3f avgV, const int idxToMirror,
				  const int offset, const float stepsize)
{
  float tmpOffset = stepsize*offset;
  cv::Vec3f tmp(0,0,pts.z);
  cv::Vec3f vrot;
  
  std::cout << std::endl;
  std::cout << "TMPOFFSET For NEW AVG " << tmpOffset << std::endl;
  std::cout << std::endl;

  tmp[idxToMirror]= -tmpOffset;
    
  vrot=rot*tmp;
  vrot[0]+=avgV[0];
  vrot[1]+=avgV[1];
  pts.x=vrot[0];
  pts.y=vrot[1];
  pts.z=vrot[2];


  return true;
}

void PredictObject::writeAsVrml(const string& file, 
				const vector<cv::Vec3f> vtx,
				const vector<int>& faces) const
{
  ofstream ofs(file.c_str());
  
    ofs << "#VRML V2.0 utf8" << endl;
    ofs << "Shape" << endl << "{" << endl;
    ofs << "geometry PointSet" << endl << "{" << endl;
    ofs << "coord Coordinate" << endl << "{" << endl;
    ofs << "point[" << endl;
    
    for(size_t i = 0; i < vtx.size(); ++i)
      ofs << vtx[i][0] << " " << vtx[i][1] << " " << vtx[i][1] << endl;
    
    ofs << "]" << endl; 
}

void PredictObject::writeAsInventor(const string& file, 
				    const vector<cv::Vec3f> vtx,
				    const vector<cv::Vec3i>& faces) 
{
  ofstream ofs(file.c_str());
  
  ofs << "#Inventor V2.1 ascii" << endl;
  ofs << "Separator {" << endl << endl;
  ofs << "Coordinate3 {" << endl;
  ofs << "point[" << endl;
  
  for(size_t i = 0; i < vtx.size(); ++i)
      ofs << vtx[i][0] << " " << vtx[i][1] << " " << vtx[i][2] << endl;
  
  ofs << "]" << endl; //point[
  ofs << "}" << endl; //Coordinate{
  
  
  ofs << "IndexedFaceSet {" << endl;
  ofs << "coordIndex[" << endl;
  
  for(size_t i = 0; i < faces.size(); ++i){
    ofs <<"   "<< faces[i][0] << ", " << faces[i][1] << ", " << faces[i][2] << " -1,"<<endl;        
  }
  ofs << "    ]" << endl; //coordIndex[
  ofs << "}" << endl; //IndexedFaceSet{
  
  
  ofs << "}" << endl; //PointSet{
  ofs <<  endl; 
}

void PredictObject::writeAsPLY(const string& file, 
			       const vector<cv::Vec3f> vtx,
			       const vector<cv::Vec3i>& faces) 
{
  ofstream ofs(file.c_str());
  
  ofs << "ply" << endl;
  ofs << "format ascii 1.0" << endl;
  ofs << "element vertex " << vtx.size() << endl;
  ofs << "property float x" << endl;
  ofs << "property float y" << endl;
  ofs << "property float z" << endl;
  ofs << "element face " << faces.size() << endl;
  ofs << "property list uchar int vertex_index" << endl;
  ofs << "end_header" << endl;

  for(size_t i = 0; i < vtx.size(); ++i)
      ofs << vtx[i][0] << " " << vtx[i][1] << " " << vtx[i][2] << endl;
  
  
  for(size_t i = 0; i < faces.size(); ++i){
    ofs <<"3 "<< faces[i][0] << " " 
	  << faces[i][1] << " " << faces[i][2] <<endl;        
  }

  ofs <<  endl; 
}

void PredictObject::ApproximateMesh(vector<cv::Point3f> &lab_cloud,
				    vector<cv::Point3f> &normals,
				    vector<float> &conf)
{
  // Set the density of the output mesh by limiting depth of tree traversal
  int depth = mDepth; 
  //Controls noise resilience by averaging over points in cell
  //float samplesPerNode=1.3;
  float samplesPerNode=17.0;
//  float samplesPerNode=1.0;
  //  take only largest segment output
  bool get_largest=true;
  
  
  CVTriangulator *_gt= new CVTriangulator(depth,samplesPerNode);
  printf("Size of normal vec %d  == %d\n",
	 (int)normals.size(),(int)lab_cloud.size());
  MeshInfo mesh_info;
/*
  for(int i=0;i<50; i++)
  printf("%f %f %f\n",
  normals[i].x,
  normals[i].y,
  normals[i].z);
    for(int i=0; i<50; i++)
    printf("%f %f %f\n",
    lab_cloud[i].x,
    lab_cloud[i].y,
    lab_cloud[i].z);
*/
  _gt->addData(lab_cloud,normals,conf);
  
  try {
    _gt->triangulate(get_largest);
    mesh_info=_gt->getMeshInfo();
  } catch (...) {
    
    cout << "Mesh Cleaning not successful" << endl;
    //    try  again without cleaning
    if(get_largest){
      delete _gt;
      _gt = new CVTriangulator(depth,samplesPerNode);
      _gt->addData(lab_cloud,normals,conf);
      try {
	_gt->triangulate(!get_largest);
      }  catch (...) {
	cout << "Mesh reconstruction not successful" << endl;
	mHavePredicted = false;
	return;
      }
    }
  }
  
  mFaces.clear();
  mFaces.reserve(_gt->_triangles.size());
  for(unsigned int i=0; i< _gt->_triangles.size(); i+=3){
    if(!(_gt->_triangles[i+0]==0 
	 && _gt->_triangles[i+1]==0 
	 && _gt->_triangles[i+2]==0)){
      cv::Vec3i f(_gt->_triangles[i+0],
		  _gt->_triangles[i+1],
		  _gt->_triangles[i+2]);
      mFaces.push_back(f);
    }
  }
  
  mFaces.resize(mFaces.size());
  mVertices.clear();
  mVertices = _gt->_vertices;
    
  char name[BUFSIZE];
  sprintf(name, "%s/meshSearch%d_%s.iv", mDirName, depth, mNoDemo);
  writeAsInventor(string(name),mVertices,mFaces);
  sprintf(name, "%s/meshSearch%d_%s.ply", mDirName, depth, mNoDemo);
  writeAsPLY(string(name),mVertices,mFaces);
  char filename[BUFSIZE];
  sprintf(filename, "%s/outputSearch%d_%s.kinbody.xml", 
	  mDirName, depth, mNoDemo);
  char meshname[BUFSIZE];
  sprintf(meshname, "meshSearch%d_%s.iv", depth, mNoDemo);
  
  DumpXML(filename, meshname, objID);

  sprintf(name, "%s/meshInfo%d_%s.txt", mDirName, depth, mNoDemo);
  DumpMeshInfo(name, mesh_info);
  
  mHavePredicted = true;
  
  cout << "Mesh reconstruction successful" << endl;
  

  delete _gt;
}

void PredictObject::SetFullSearch(bool full_search)
{
    full_search_ = full_search;
}

bool PredictObject::GetFullSearch()
{
    return full_search_;
}

void PredictObject::GetMinorAxis(cv::Point3f &lMinor)
{
  lMinor.x = eigenVectors->data.fl[2];
  lMinor.y = eigenVectors->data.fl[3];
  lMinor.z = 0;
}

void PredictObject::GetMean(cv::Point3f &lMean)
{
 
  CalcCentroid(mCloud, lMean);
  lMean.z = maxZ;

}



void PredictObject::GetMeanAllAxis(cv::Point3f &lMean)
{
 
  CalcCentroid(mCloud, lMean);
}

void PredictObject::GetMirMean(cv::Point3f &lMean)
{
  // does not give mean Z but max Z
  cv::Point2f minmaxX, minmaxY, minmaxZ;
  GetMinMaxPoints( mMirCloud, minmaxX, minmaxY, minmaxZ ); 
  cv::Point3f newCenter;
  CalcCentroid(mMirCloud, newCenter);
  lMean.x = newCenter.x;
  lMean.y = newCenter.y;
  lMean.z = minmaxZ.y;

}

void PredictObject::GetMirMeanAllAxis(cv::Point3f &lMean)
{
  CalcCentroid(mMirCloud, lMean);

}

void PredictObject::GetPlausibility(float &plausibility)
{
  plausibility = 1.0f-bestVote;
}

void PredictObject::GetPlausibility(std::vector<float> &plausibility)
{
  plausibility = mPlausibilities;
}

void PredictObject::GetNormals(std::vector<cv::Point3f> &lNormals)
{
  lNormals = mNormals;
}

void PredictObject::GetOriginal(std::vector<bool> &lOriginal)
{
  lOriginal = mOriginal;
}

void PredictObject::GetMesh(std::vector<cv::Vec3f> &lVertices,
			    std::vector<cv::Vec3i> &lTriangles)
{
  lVertices = mVertices;
  lTriangles = mFaces;
}

void PredictObject::DumpPlaneParams( const char *filename)
{
    FILE *lFILE=fopen(filename,"w");
    fprintf(lFILE,"%f %f %f\n", 
	    mPlanePoint.x, 
	    mPlanePoint.y, 
	    mPlanePoint.z);
    fprintf(lFILE,"%f %f %f\n", 
	    mPlaneNormal[0], 
	    mPlaneNormal[1], 
	    mPlaneNormal[2]);
    fclose(lFILE);

}

void PredictObject::DumpPointCloud( const char *filename )
{
  FILE *lFILE=fopen(filename,"w");
  fprintf(lFILE,"%ld\n",(long int)mCloud.size());
  
  
  for(vector<cv::Point3f>::iterator i=mCloud.begin();i!=mCloud.end();++i) {
    fprintf(lFILE,"%f %f %f %i %i %i %i %i\n",
	      i->x, i->y, i->z, 0, 0, 0, 0, 0);
  } 
  
  fclose(lFILE);
}

void PredictObject::DumpMirroredCloud( const char *filename, vector<cv::Point3f> &lab_cloud )
{
  FILE *lFILE=fopen(filename,"w");
  fprintf(lFILE,"%ld\n",(long int)lab_cloud.size());
  
  for(vector< cv::Point3f>::iterator i=lab_cloud.begin();
      i!=lab_cloud.end();++i) {
    fprintf(lFILE,"%f %f %f %i %i %i %i %i\n", i->x, i->y, i->z, 0, 0, 0, 0, 0);
  } 
  
  fclose(lFILE);
}


void PredictObject::DumpMirroredCloud( const char *filename, 
				       vector<cv::Point3f> &lab_cloud,
				       vector<float> &plausibilities)
{
  FILE *lFILE=fopen(filename,"w");
  fprintf(lFILE,"%ld\n",(long int)lab_cloud.size());
  
  int idx = 0;
  for(vector< cv::Point3f>::iterator i=lab_cloud.begin();
      i!=lab_cloud.end();++i) {
    unsigned char grey = plausibilities.at(idx) * 255.0f;
    fprintf(lFILE,"%f %f %f %i %i %i %i %i\n", i->x, i->y, i->z, 0, 0, grey, grey, grey);
    idx++;
  } 
  
  fclose(lFILE);
}


void PredictObject::DumpMeshInfo( const char *filename, 
				  MeshInfo mesh_info)
{
    FILE *lFILE=fopen(filename,"w");
    fprintf(lFILE,"%i %i %i %i\n", 
	    mesh_info.boundary_loops_,
	    mesh_info.boundary_vertices_,
	    mesh_info.connected_components_,
	    mesh_info.manifold_);
    fclose(lFILE);
}
