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

#ifndef UTILITIES_H
#define UTILITIES_H
#include <opencv2/opencv.hpp>
#include <vector>
#include "vox.h"

struct Leaf {
  std::vector<int> pts;
};

inline float square( float x)
{
  return x*x;
};

template <typename T> 
inline T RadToDeg(T val) { 
  return val*180.0/3.1415; 
}

template <typename T> 
inline T DegToRad(T val) { 
  return val*3.1415/180.0; 
}

void FilterSegment( std::vector<Vox> &pCloud, cv::Point3f leaf_size, int num_pts,
		    cv::Point2f &minmaxX, cv::Point2f &minmaxY, cv::Point2f &minmaxZ );
void FilterSegment( std::vector<cv::Point3f> &pCloud, cv::Point3f leaf_size, int num_pts,
		    cv::Point2f &minmaxX, cv::Point2f &minmaxY, cv::Point2f &minmaxZ );
void GetMinMaxPoints( std::vector<Vox> &pCloud, 
		      cv::Point2f &minmaxX, cv::Point2f &minmaxY, cv::Point2f &minmaxZ ); 
void GetMinMaxPoints( std::vector<cv::Point3f> &pCloud, 
		      cv::Point2f &minmaxX, cv::Point2f &minmaxY, cv::Point2f &minmaxZ );
void ComputeNormals(std::vector<cv::Point3f> lPoints, std::vector<cv::Point3f> lNormals);

float CalcLength( CvPoint2D32f lP1,  CvPoint2D32f lP2);
float CalcLength( CvPoint3D32f lP1,  CvPoint3D32f lP2);
void InvertImage(const cv::Mat &src, cv::Mat &dst);

void DumpXML( const char *filename, const char *meshname, int objID);

float MapRanges( float origMin,
		 float origMax,
		 float targetMin,
		 float targetMax,
		 float val);

void normalize(cv::Vec3f &v);

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Flip (in place) the estimated normal of a point towards a given viewpoint
    * \param normal the plane normal to be flipped
    * \param point a given point
    * \param viewpoint the viewpoint
    */
inline void
flipNormalTowardsViewpoint (cv::Point3f &normal, cv::Point3f point, cv::Point3f viewpoint)
{
  // printf("%f %f %f\n",normal.x,normal.y,normal.z);
  // See if we need to flip any plane normals
    float vp_m[3];
    vp_m[0] = viewpoint.x - point.x;
    vp_m[1] = viewpoint.y - point.y;
    vp_m[2] = viewpoint.z - point.z;
    
    // Dot product between the (viewpoint - point) and the plane normal
    float cos_theta = (vp_m[0] * normal.x+ vp_m[1] * normal.y + vp_m[2] * normal.z);
    
    // Flip the plane normal
    if (cos_theta < 0)
      {
        normal.x *= -1;
        normal.y *= -1;
        normal.z *= -1;
	
      }
};

#endif
