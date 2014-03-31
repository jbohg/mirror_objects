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

/* The Functions called FilterSegment are adapted from 
 * the PCL voxel grid filter (voxel_grid.h) retaining only the 
 * placing of the 3D grid and outlier removal. Functions are stripped
 * to keep no dependencies on PCL or Eigen. OpenCV dependencies used 
 * instead.
 * 
 * Add link to website
 */

#include "utilities.h"
#include <stdio.h>
#include <opencv2/contrib/contrib.hpp>

void FilterSegment(std::vector<Vox> &pCloud, 
		   cv::Point3f leaf_size, 
		   int num_pts,
		   cv::Point2f &minmaxX, 
		   cv::Point2f &minmaxY, 
		   cv::Point2f &minmaxZ){
  
  std::vector<Leaf>leaves;
  std::vector<int> indices2;
  for(unsigned int k=0; k < pCloud.size(); k++)
    if( pCloud[k].id != 0){
      indices2.push_back(k);
    }
  
  cv::Point3f min_b, max_b, div_b;
  float minX = minmaxX.x;
  float maxX = minmaxX.y;
  
  float minY = minmaxY.x;
  float maxY = minmaxY.y;
  
  float minZ = minmaxZ.x;
  float maxZ = minmaxZ.y;

  // Compute the minimum and maximum bounding box values
  min_b.x = (int)(floor (minX / leaf_size.x));
  max_b.x = (int)(floor (maxX / leaf_size.x));
  
  min_b.y = (int)(floor (minY / leaf_size.y));
  max_b.y = (int)(floor (maxY / leaf_size.y));
  
  min_b.z = (int)(floor (minZ / leaf_size.z));
  max_b.z = (int)(floor (maxZ / leaf_size.z));
  
  // Compute the number of divisions needed along all axis
  div_b.x = (int)(max_b.x - min_b.x + 1);
  div_b.y = (int)(max_b.y - min_b.y + 1);
  div_b.z = (int)(max_b.z - min_b.z + 1);

  // Allocate the space needed
  try {
    if (leaves.capacity () < div_b.x * div_b.y * div_b.z)
      leaves.reserve (div_b.x * div_b.y * div_b.z);             
    leaves.resize (div_b.x * div_b.y * div_b.z);
  } catch (std::bad_alloc) {
    printf("Failed while attempting to allocate a vector of %f (%g x %g x %g) leaf elements (%f bytes total)", div_b.x * div_b.y * div_b.z, div_b.x, div_b.y, div_b.z, div_b.x * div_b.y * div_b.z * sizeof (Leaf));
  }
  unsigned long leaves_size = div_b.x * div_b.y * div_b.z;
  
  // First pass: go over all points and insert them into the right leaf
  for (unsigned int cp = 0; cp < indices2.size (); cp++) {
    int i = (int)(floor (pCloud[indices2.at (cp)].pos.x / leaf_size.x));
    int j = (int)(floor (pCloud[indices2.at (cp)].pos.y / leaf_size.y));
    int k = (int)(floor (pCloud[indices2.at (cp)].pos.z / leaf_size.z));
    
    int idx = ( (k - min_b.z) * div_b.y * div_b.x ) + ( (j - min_b.y) * div_b.x ) + (i - min_b.x);
    leaves[idx].pts.push_back(cp);
  }

  // Second pass: go over all leaves and check if they contain minimum number of points
  for (unsigned int cl = 0; cl < leaves_size; cl++) {
    int sizeLeaf = (int)leaves[cl].pts.size();
    
    if (sizeLeaf < num_pts) {
      for(int c=0; c < sizeLeaf; c++){
	int cp = leaves[cl].pts[c];
	pCloud[indices2.at (cp)].id = 0;
      }
    } 
  }
}

void FilterSegment(std::vector<cv::Point3f> &pCloud, 
		   cv::Point3f leaf_size, int num_pts,
		   cv::Point2f &minmaxX, 
		   cv::Point2f &minmaxY, 
		   cv::Point2f &minmaxZ){
  
  std::vector<Leaf>leaves;
  std::vector<int> indices2;
  indices2.reserve(pCloud.size());
  indices2.resize(pCloud.size(),0);
  for(unsigned int k=0; k < pCloud.size(); k++)
    indices2.at(k)=k;

  std::vector<cv::Point3f> lCloud; 
  
  cv::Point3f min_b, max_b, div_b;
  float minX = minmaxX.x;
  float maxX = minmaxX.y;
  
  float minY = minmaxY.x;
  float maxY = minmaxY.y;
  
  float minZ = minmaxZ.x;
  float maxZ = minmaxZ.y;

  // Compute the minimum and maximum bounding box values
  min_b.x = (int)(floor (minX / leaf_size.x));
  max_b.x = (int)(floor (maxX / leaf_size.x));
  
  min_b.y = (int)(floor (minY / leaf_size.y));
  max_b.y = (int)(floor (maxY / leaf_size.y));
  
  min_b.z = (int)(floor (minZ / leaf_size.z));
  max_b.z = (int)(floor (maxZ / leaf_size.z));
  
  // Compute the number of divisions needed along all axis
  div_b.x = (int)(max_b.x - min_b.x + 1);
  div_b.y = (int)(max_b.y - min_b.y + 1);
  div_b.z = (int)(max_b.z - min_b.z + 1);

  // Allocate the space needed
  try {
    if (leaves.capacity () < div_b.x * div_b.y * div_b.z)
      leaves.reserve (div_b.x * div_b.y * div_b.z);             
    leaves.resize (div_b.x * div_b.y * div_b.z);
  } catch (std::bad_alloc) {
    printf("Failed while attempting to allocate a vector of %f (%g x %g x %g) leaf elements (%f bytes total)", div_b.x * div_b.y * div_b.z, div_b.x, div_b.y, div_b.z, div_b.x * div_b.y * div_b.z * sizeof (Leaf));
  }
  unsigned long leaves_size = div_b.x * div_b.y * div_b.z;
  
  // First pass: go over all points and insert them into the right leaf
  for (unsigned int cp = 0; cp < indices2.size (); cp++) {
    int i = (int)(floor (pCloud[indices2.at (cp)].x / leaf_size.x));
    int j = (int)(floor (pCloud[indices2.at (cp)].y / leaf_size.y));
    int k = (int)(floor (pCloud[indices2.at (cp)].z / leaf_size.z));
    
    int idx = ( (k - min_b.z) * div_b.y * div_b.x ) + ( (j - min_b.y) * div_b.x ) + (i - min_b.x);
    leaves[idx].pts.push_back(cp);
  }

  // Second pass: go over all leaves and check if they contain minimum number of points
  for (unsigned int cl = 0; cl < leaves_size; cl++) {
    int sizeLeaf = (int)leaves[cl].pts.size();
    
    if (sizeLeaf > num_pts) {
      for(int c=0; c < sizeLeaf; c++){
	int cp = leaves[cl].pts[c];
	lCloud.push_back(pCloud[indices2.at (cp)]);
      }
    } 
  }
  
  pCloud = lCloud;

}


void GetMinMaxPoints( std::vector<Vox> &pCloud, 
		      cv::Point2f &minmaxX, 
		      cv::Point2f &minmaxY, 
		      cv::Point2f &minmaxZ )
{
  float minXCp=0.0f, maxXCp=0.0f, 
    minYCp=0.0f, maxYCp=0.0f, minZCp=0.0f, maxZCp=0.0f;
  int count = 0;
  bool first = true;
  for ( unsigned int k=0; k<pCloud.size(); ++k) {
    if( pCloud[k].id != 0){
      count ++;
      
      if (!first){
	if(pCloud[k].pos.x>maxXCp)
	  maxXCp=pCloud[k].pos.x;
	else if (pCloud[k].pos.x<minXCp)
	  minXCp=pCloud[k].pos.x;
	
	if(pCloud[k].pos.y>maxYCp)
	  maxYCp=pCloud[k].pos.y;
	else if (pCloud[k].pos.y<minYCp)
	  minYCp=pCloud[k].pos.y;
	
	if(pCloud[k].pos.z>maxZCp)
	  maxZCp=pCloud[k].pos.z;
	else if (pCloud[k].pos.z<minZCp)
	  minZCp=pCloud[k].pos.z;
	
      } else {
	maxXCp=pCloud[k].pos.x;
	minXCp=pCloud[k].pos.x;
	maxYCp=pCloud[k].pos.y;
	minYCp=pCloud[k].pos.y;
	maxZCp=pCloud[k].pos.z;
	minZCp=pCloud[k].pos.z;
	first = false;
      }
    }
  }

  minmaxX.x = minXCp; 
  minmaxX.y = maxXCp; 
  minmaxY.x = minYCp; 
  minmaxY.y = maxYCp; 
  minmaxZ.x = minZCp; 
  minmaxZ.y = maxZCp;

}

void GetMinMaxPoints( std::vector<cv::Point3f> &pCloud, 
		      cv::Point2f &minmaxX, 
		      cv::Point2f &minmaxY, 
		      cv::Point2f &minmaxZ )
{
  float minXCp=0.0f, 
    maxXCp=0.0f, 
    minYCp=0.0f, 
    maxYCp=0.0f, 
    minZCp=0.0f, 
    maxZCp=0.0f;

  bool first = true;
  for (unsigned int k=0; k<pCloud.size(); ++k) {
    
    if (!first){
      if(pCloud[k].x>maxXCp)
	maxXCp=pCloud[k].x;
      else if (pCloud[k].x<minXCp)
	minXCp=pCloud[k].x;
      
      if(pCloud[k].y>maxYCp)
	maxYCp=pCloud[k].y;
      else if (pCloud[k].y<minYCp)
	minYCp=pCloud[k].y;
      
      if(pCloud[k].z>maxZCp)
	maxZCp=pCloud[k].z;
      else if (pCloud[k].z<minZCp)
	minZCp=pCloud[k].z;
      
    } else {
      maxXCp=pCloud[k].x;
      minXCp=pCloud[k].x;
      maxYCp=pCloud[k].y;
      minYCp=pCloud[k].y;
      maxZCp=pCloud[k].z;
      minZCp=pCloud[k].z;
      first = false;
    }
  }

  minmaxX.x = minXCp; 
  minmaxX.y = maxXCp; 
  minmaxY.x = minYCp; 
  minmaxY.y = maxYCp; 
  minmaxZ.x = minZCp; 
  minmaxZ.y = maxZCp;

}

void ComputeNormals(std::vector<cv::Point3f> lPoints, 
		    std::vector<cv::Point3f> lNormals)
{
  cv::Mesh3D mesh( lPoints);
  mesh.computeNormals(10.0);
  lNormals.reserve(mesh.normals.size());
  lNormals.resize(mesh.normals.size());
  for(unsigned int i=0; i < mesh.normals.size();i++){
    lNormals.at(i)=mesh.normals[i];
  }
}

float CalcLength( CvPoint2D32f lP1,  CvPoint2D32f lP2)
{
  float dist = sqrt(square(lP2.x-lP1.x) + square(lP2.y-lP1.y));
  return dist;
}

float CalcLength( CvPoint3D32f lP1,  CvPoint3D32f lP2)
{
  float dist = sqrt(square(lP2.x-lP1.x) + square(lP2.y-lP1.y) + square(lP2.z-lP1.z));
  return dist;
}

void InvertImage( const cv::Mat &src, cv::Mat &dst)
{
  dst.create(src.size(),CV_8U);
  for(int x=0; x<src.rows; x++)
    for(int y=0; y<src.cols; y++)
      dst.at<uchar>(x, y) = 255-src.at<uchar>(x, y);
}

void DumpXML( const char *filename, const char *meshname, int objID)
{
  FILE *lFILE=fopen(filename,"w");
  if (lFILE!=NULL) {
    fprintf(lFILE,"<KinBody name=\"output%d\">\n", objID);
    fprintf(lFILE,"\t<RotationAxis>1 0 0 0</RotationAxis>\n");
    fprintf(lFILE,"\t<Body type=\"dynamic\">\n");
    fprintf(lFILE,"\t\t<Geom type=\"trimesh\">\n");
    fprintf(lFILE,"\t\t\t<Render>%s 0.001</Render>\n", meshname);
    fprintf(lFILE,"\t\t\t<Data>%s 0.001</Data>\n", meshname);
    fprintf(lFILE,"\t\t</Geom>\n");     
    fprintf(lFILE,"\t</Body>\n"); 
    fprintf(lFILE,"</KinBody>\n"); 
    
    fclose(lFILE);
  } else {
    std::cout << "Mesh could not be written. Check if target directory exists!" << std::endl;
    exit(-1);
  } 
}

float MapRanges( float origMin,
		 float origMax,
		 float targetMin,
		 float targetMax,
		 float val)
{
  return (val-origMin)/(origMax-origMin) * (targetMax-targetMin) 
    + targetMin; 
}



void normalize(cv::Vec3f &v){
  float normalize = 0;
  for( int kk = 0; kk < 3; kk++ )
    normalize += v[kk]*v[kk];
  normalize = 1./(sqrt(normalize) + DBL_EPSILON);
  for( int kk = 0; kk < 3; kk++ )
    v[kk] = (float)(v[kk]*normalize);
}



