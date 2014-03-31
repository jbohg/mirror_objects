/*********************************************************************
*
*  Copyright (c) 2011, Jeannette Bohg, Matthew Johnson-Roberson 
*  - KTH Stockholm ({bohg,mattjr}@csc.kth.se)
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
*   * Neither the name of Jeannette Bohg, Matthew Johnson-Roberson
*     nor the names of KTH
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
/* 
	The interface to OpenCV is based on OSG Interface by 
	Martin Beckett 2009.

	The clever code is: Copyright (c) 2006, Michael Kazhdan 
	and Matthew Bolitho
	With a MIT/BSD licence - see details in the poisson files.	
	http://www.cs.jhu.edu/~misha/Code/PoissonRecon/
 
*/

#ifndef CVTRIANGULATOR_H
#define CVTRIANGULATOR_H



#include <opencv2/opencv.hpp>
#include <vector>
#include "boundingbox.h"
class CoredVectorMeshData;
template<int Degree> class Octree;

/** CVTriangulator: Utility class that triangulates an irregular 3D set of sample points.
    Uses an Octree to divide the space up into a 3D grid and solves Poisson equation to create mesh	
    Create CVTriangulator() and set the maximum depth of the Octree - larger depth is more detail but more time and memory
    addData() vertex and normals individually or as geode, the solver uses the Normals to determine which side is inside a model.
    If no normals are supplied it can mesh a 2d surface by assuming that the Normal is up.
    The octree rescales data into a range 0-1 so it needs to have the overall bounds of all data first.
    If not using a Geode and supplying multiple sets of primatives you must set the overall boundign box first.
    
    Call triangulate() method to start the triangulation. 
    Then you can obtain the generated primitive by calling the getPoints() getTriangles() method.
    
    Nice features: 
    Fast and uses low amounts of memory
    Produces watertight 3D models and 2d mdoels with no weird triangles
    Adaptive detail based on degree of detial of local surface
    Output points are on a regular grid so ideal for generating DEM/contours/stripes
    
    TODO:
    Extends a 2d surface to the bounding box so we need to add a boundary constraint to chop off the outer triangles.
    Not sure if this should be part of this class or a general polygon/boundary trimmer
    (since the output is on a grid there are a bucnh of quick in-polygon algorithms)
    Need some way of getting an update inside the octree loop to update a progress bar - do we just pass a function pointer ?
    
    Notes:
    I have changed the original code as little as possible, a couple of methods have been made public 
    The octree uses it's own points/triangle classes, I haven't changed these to osg types because it wouldn't gain anything 
    and even as cut-paste would probably introduce errors. 
    
    The octree solver code is templated for any type and for any number of dimensions. This means that there is a .inl file for each .h 
    and it uses Real as a typedef for double/float.
    
*/

// Help - I don't really know if this should derive from Referenced but DelaunayTriangulator does!
//namespace osgUtil 
//{
//class OSGUTIL_EXPORT CVTriangulator: public osg::Referenced {

struct MeshInfo
{
    
    MeshInfo();
   
    int boundary_loops_;
    int boundary_vertices_;
    int connected_components_;
    
    bool manifold_;

};
    
class CVTriangulator {
 public:
  
  /** Depth is the max level of detail in the tree, default 8.*/
  CVTriangulator(const int depth=8, const float samplesPerNode=1.0f);
  
  /** Uses all the point and normals data in the geode, can also specify depth.*/
  //CVTriangulator(const osg::Geode *geode,const int depth=8);
  
  /** Need to cleanup the allocated mesh and tree */
  ~CVTriangulator(); // HELP - what is the benefit in making this protected as in DelaunayTriangulator? 
  
  /** If using multiple sets of data the overall bound must be knwon in advance. The solver needs to prescale the data*/
  bool setBound(const BoundingBox& box);
  
  /** Number of points to use for each sample. Default is 1, increase for noisy data*/
  bool setSamples(const float samples) { samplesPerNode=samples; return true; }
  
  /** Uses all the point and normals data in the geode, calcualtes the bounding box*/
  
  bool addData(std::vector<cv::Point3f> points,
	       std::vector<cv::Point3f> norms,
	       std::vector<float> conf);
  
  /** Start triangulation. (TODO - need to split this up or pass a callback so can update a progress bar)*/
  bool triangulate(bool get_largest=true);
  
  MeshInfo getMeshInfo();
  
  // Naming is to match the octree code
  cv::Vec3f center;
  float scale;
  std::vector<cv::Vec3f> _vertices;
  std::vector<int> _triangles;

 protected:
  
  
  
  
  int maxDepth; // depth of octree, default 8. greater depth is slower and more memeory but more detail
  
  // These are pointers because otherwise I need to include MultiGridOctree.h which includes dozens of static classes and the templates screw up VS2008
  Octree<2> *tree; // opaque structure to store the input data 
  CoredVectorMeshData *mesh; // opaque structure to store the result
  
  // the following are tuning parameters that are unchanged from the defaults in the original code
  float scaleFactor; // scales between model extents and octree solution, default 1.25
  int refine; // default 3 
  int solverDivide; // at depths greater than this use a slower/less memory intensive algorithm for solving step.
  float samplesPerNode; // number of points to use for each fit. Default 1, increase for noisy data
 protected:
  
  //  	osg::ref_ptr<osg::DrawElementsUInt> _triangles;
  //	osg::ref_ptr<osg::Geode> _octreeGeode;
  //osg::ref_ptr<osg::Geometry> geometry;
  std::vector<cv::Vec3f>_normals;
  
private:
  
  MeshInfo mesh_info_;

};

#endif

