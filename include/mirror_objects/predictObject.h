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
#ifndef PREDICTOBJECT_H
#define PREDICTOBJECT_H

#include <vector>
#include <opencv2/opencv.hpp>
#include "quat.hpp"
#include "poisson/CVTriangulator.h"

class PredictObject 
{
  
public:
  
  struct Vote{
    
    float divergeVal;
    float overlapVal;

    float divergeMedian;
    float overlapMedian;
    
    int nPix;
    int nPoints;

    float val;
    
  };

  PredictObject();

  PredictObject( std::vector<cv::Point3f> lCloud,
		 int id,
		 cv::Point3f lOrig,
		 int lDepth,
		 bool lSparse,
		 cv::Point2d lFocal,
		 cv::Point2d lCentre,
		 cv::Mat lMask,
		 const char* lDirName,
		 const char* lNoDemo);

  PredictObject( std::vector<cv::Point3f> lCloud,
		 int id,
		 cv::Mat lTrans,
		 int lDepth,
		 bool lSparse, 
		 cv::Point2d lFocal,
		 cv::Point2d lCentre,
		 cv::Mat lMask,
		 const char* lDirName,
		 const char* lNoDemo);

  PredictObject( std::vector<cv::Point3f> lCloud,
		 std::vector<cv::Point3f> normals,
		 std::vector<float> plausibilities,
		 int id,
		 int lDepth,
		 bool use_plausibilities,
		 const char* lDirName,
		 const char* lNoDemo);
  
  ~PredictObject();

  void Mirror( bool filter );
  void ConstructMesh();
  void MirrorFromPlane( const std::vector<cv::Point3f> &lCloud,
			const cv::Point3f lOrig,
			const cv::Point3f lC,
			const cv::Vec3f   lMirrAxis,
			std::vector<cv::Point3f> &lOrigCloud, 
			std::vector<cv::Point3f> &lMirCloud,
			std::vector<cv::Point3f> &normals,
			std::vector<bool> &original);
  void SetFullSearch(bool full_search);
  bool GetFullSearch();
  int  GetID();
  bool GetMirCloud(std::vector<cv::Point3f> &lMirCloud);
  void GetMinorAxis(cv::Point3f &lMinor);
  void GetMean(cv::Point3f &lMean);
  void GetMeanAllAxis(cv::Point3f &lMean);
  void GetMirMean(cv::Point3f &lMean);
  void GetMirMeanAllAxis(cv::Point3f &lMean);
  void GetPlausibility(float &plausibility);
  void GetPlausibility(std::vector<float> &plausibility);
  void GetNormals(std::vector<cv::Point3f> &lNormals);
  void GetOriginal(std::vector<bool> &lOriginal);
  void GetMesh(std::vector<cv::Vec3f> &lVertices,
	       std::vector<cv::Vec3i> &lTriangles);
  void DumpPlaneParams( const char *filename);
  bool HavePredicted();
  bool HaveMirrored();

  void DumpObjectAsIV( const char *filename );
  void DumpPointCloud( const char *filename );
  void DumpMirroredCloud( const char *filename, 
			  std::vector<cv::Point3f> &lab_cloud );
  void DumpMirroredCloud( const char *filename, 
			  std::vector<cv::Point3f> &lab_cloud,
			  std::vector<float> &plausibilities);
  void DumpMeshInfo( const char *filename, 
		     MeshInfo mesh_info);
  void DumpOrigMesh( const char *filename );
    
 private:
  cv::flann::Index CreateLookUpData(const std::vector<cv::Point3f> &lCloud,
				    const cv::Vec3f avg3D,
				    cv::Mat &lDepth, cv::Mat &lMask,
				    cv::Mat &lPoints);
  void MakeDepthMap(const std::vector<cv::Point3f> &lCloud, 
		    const cv::Vec3f avg3D, 
		    cv::Mat &lDepth, 
		    cv::Mat &lMask, 
		    cv::Mat &lPoints);
  cv::flann::Index MakeFLANNIndex( const cv::Mat &lPoints);
    
  void ProjectOnTable( std::vector<cv::Point3f> &lCloud, 
		       cv::Mat &lImg, 
		       float minX, 
		       float maxX, 
		       float minY, 
		       float maxY, 
		       bool filter );
  void MirrorCloud(const cv::Mat &lPoints, 
		   cv::flann::Index &lFLANNIdx,
		   const cv::Vec3f avg3D,
		   std::vector<cv::Point3f> &lab_cloud, 
		   std::vector<cv::Point3f> &lMirCloud,
                   std::vector<cv::Point3f> &normals,
		   std::vector<float> &plausibilities,
		   std::vector<bool> &original);
  void MirrorCloudSearchOrient(const cv::Mat &lPoints, 
			       cv::flann::Index &lFLANNIdx,
			       const cv::Vec3f avg3D,
			       std::vector<cv::Point3f> &lab_cloud, 
			       std::vector<cv::Point3f> &lMirCloud,
			       std::vector<cv::Point3f> &normals,
			       std::vector<float> &plausibilities,
			       std::vector<bool> &original);
  void MirrorShift(std::vector<std::vector<cv::Point3f> > &lMirroredClouds,
		   std::vector<std::vector<cv::Point3f> > &lOrigClouds,
		   std::vector<Vote> &lVotes,
		   std::vector<cv::Point3f> &lRotOrigs,
		   std::vector<float> &lStepSizes,
		   std::vector<std::vector<std::pair<bool,float> > > &lPlausibilities,
		   const cv::Mat &lPoints, 
		   cv::flann::Index &lFLANNIdx,
		   const cv::Vec3f avg3f,
		   const cv::Vec3f avgV, 
		   const cv::Vec3f viewDir, 
		   const cv::Vec3f MirrAxis, 
		   const cv::Vec3f PerpAxis, 
		   const Quat rot, 
		   const int idxToMirror,
		   float &shift3D, 
		   float &stepsize,
		   cv::Mat *rgbProj=NULL);
  bool MirrorPoint(const cv::Point3f pts, cv::Point3f &mirpts,
		   const Quat rot, 
		   const cv::Vec3f avgV, 
		   const int idxToMirror, 
		   float &minIdxToMirror, 
		   float &maxIdxToMirror,
		   const int offset, 
		   const float stepsize,
		   cv::Mat &lProj);
  bool MirrorPoint(const cv::Point3f pts, 
		   cv::Point3f &mirpts,
		   const Quat lRot,
		   const cv::Point3f lC);
  bool DemirrorPoint(cv::Point3f &pts,
		     const Quat rot, const cv::Vec3f avgV, const int idxToMirror,
		     const int offset, const float stepsize);
  void GetFixedSurfaceParameters(cv::Vec3f &avgV,
				 cv::Vec3f &viewDir,
				 cv::Vec3f &lMirrAxis, 
				 cv::Vec3f &lPerpAxis,
				 Quat &lRot, 
				 int &idxToMirror);

  bool ProjectPoint(const float x, const float y, const float z, 
		    const cv::Mat &lMask,
		    cv::flann::Index *lFLANNIdx, 
		    const cv::Mat &lPoints,
		    cv::Mat &lImg, 
		    cv::Mat &lOver, 
		    cv::Mat &lDepth,
		    std::pair<bool,float> &plaus);
  
  bool ProjectPoint(const float x, const float y, 
		    const float z, const cv::Mat &lMask,
		    cv::flann::Index *lFLANNIdx, 
		    const cv::Mat &lPoints,
		    cv::Mat &lImg, cv::Mat &lOver, 
		    cv::Mat &lNImg, cv::Mat &lNOver, 
		    cv::Mat &lDepth,
		    std::pair<bool,float> &plaus);

  void ProjectPoints(const std::vector<cv::Point3f> &lCloud, 
		     const cv::Mat lRotMat, 
		     cv::Mat &lDepth, 
		     cv::Mat &lMask, 
		     cv::Mat &lPoints);
  void Arm2Cam(const std::vector<cv::Point3f> &lCloud, 
	       const cv::Mat lRotMat, cv::Mat &lRotCloudTrans);
  void ProjectPoints( const std::vector<cv::Point3f> &lCloud, 
		      const Quat rot,  
		      cv::Mat &lImg, 
		      cv::Mat &lMask, 
		      cv::Mat &lPoints);
  void Arm2Cam(const cv::Point3f pts, 
	       const Quat rot, float *x, float *y, float *z);
  void FillProjection(cv::Mat &lSrc, cv::Mat &lDst, bool sparse);

  void ProjectOnImage( std::vector<cv::Point3f> &lCloud, 
		       const cv::Mat &lMask, 
		       const cv::Vec3f avg3D,
		       cv::flann::Index *lFLANNIdx, 
		       const cv::Mat &lPoints,
		       cv::Mat &lImg, 
		       cv::Mat &lOver, 
		       cv::Mat &lDepth,
		       std::vector<std::pair<bool,float> > &lPlaus);
  void Divergence2Plausability(std::vector<std::pair<bool,float> > &lPlaus);
  Vote GetVote( const cv::Mat &lImg, const cv::Mat &lOver);
  void CalcVotes( std::vector<Vote> &lVotes, 
		  float maxShift3D = 0, 
		  float lWDiverge = 0.5);
  void CalcVotes( std::map<std::pair<int,int>,Vote> &lVotes,
		  float lWDiverge = 0.5);
  int SelectBestCloud(const std::vector<Vote> lVotes);
  std::pair<int,int> SelectBestCloud(const std::map< std::pair<int, int>, Vote> lVotes);
  void CalcPCA( bool calcCH= false );
  void CalcCentroid(const cv::Mat &lProj, cv::Point2d &lCentroid);
  void CalcCentroid(const std::vector<cv::Point3f> lCloud, 
		    cv::Point3f &lAvg);
  void ApproximateMesh(std::vector<cv::Point3f> &lab_cloud,
		       std::vector<cv::Point3f> &normals,
      		       std::vector<float> &conf);
  void writeAsVrml(const std::string& file, 
		   const std::vector<cv::Vec3f> vtx,
		   const std::vector<int>& faces) const;

  
  void writeAsInventor(const std::string& file, 
		       const std::vector<cv::Vec3f> vtx,
		       const std::vector<cv::Vec3i>& faces);

  void writeAsPLY(const std::string& file, 
		  const std::vector<cv::Vec3f> vtx,
		  const std::vector<cv::Vec3i>& faces) ;
  cv::Mat     mProj;
  std::vector<cv::Point3f> mCloud;
  std::vector<cv::Point3f> mMirCloud;
  std::vector<cv::Point3f> mNormals;
  std::vector<float> mPlausibilities;
  std::vector<bool> mOriginal;
  
  std::vector<cv::Vec3f> mVertices;
  std::vector<cv::Vec3i> mFaces;

  CvMat* eigenVectors;
  CvMat* eigenValues;
  CvMat* avg;

  cv::Point3f mPlanePoint;
  cv::Vec3f   mPlaneNormal;

  cv::Point3f mirrMean;

  int objID;

  cv::Point3f orig;
  cv::Mat     mTrans;
  cv::Mat     mMask;
  cv::Mat     mDepthLU;

  float minX, maxX;
  float minY, maxY;
  float minZ, maxZ;

  float maxDiverge;
  int nDiverge;
  float maxOverlap;
  int nOverlap;
  float maxHidden;
  int nHidden;

  float bestVote;

  bool mHavePredicted;
  bool mHaveMirrored;
  bool mHaveTrans;
  bool mUseConfidence;

  bool full_search_;

  bool mSparse;
  cv::Mat mImg;
  cv::Mat mDistTrans;

  int mDepth;
  cv::Point2d mFocal;
  cv::Point2d mCentre;
  
  float orig_plaus;

  char* mDirName;
  char* mNoDemo;

};

#endif // PREDICTOBJECT_H
