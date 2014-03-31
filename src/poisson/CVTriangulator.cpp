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


#include "Clean.h"
#include "PPolynomial.h"
#include "BinaryNode.h"
#include "FunctionData.h"
#include "MultiGridOctreeData.h"
#include <iostream>
#include "CVTriangulator.h"

// Which namespace shoudl this go in?
//namespace osgUtil
//{
using namespace std;


MeshInfo::MeshInfo() 
 : boundary_loops_(-1)
 , boundary_vertices_(-1)
 , connected_components_(-1)
 , manifold_(true)
{}

CVTriangulator::CVTriangulator(const int depth, 
			       const float samplesPerNode)
    : scale(0.0f)
    , scaleFactor(1.25f)
    , refine(3)
    , solverDivide(8)
    , samplesPerNode(samplesPerNode)	
{
    maxDepth=depth;
    mesh=NULL;
    /*geometry = new osg::Geometry;
      geometry->setDataVariance( osg::Object::DYNAMIC );
    */
    TreeNodeData::UseIndex=1;
    PPolynomial<2> ReconstructionFunction
	=PPolynomial<2>::GaussianApproximation();
    TreeOctNode::SetAllocator(MEMORY_ALLOCATOR_BLOCK_SIZE);
    
    tree = new Octree<2>;
    tree->setFunctionData(ReconstructionFunction,
			  maxDepth,
			  0,
			  float(1.0)/(1<<maxDepth));	
    tree->neighborKey.set(maxDepth);	
    tree->normals=new std::vector<Point3D<Real> >();
    
}


CVTriangulator::~CVTriangulator()
{
    delete tree->normals;
    delete tree;
    delete mesh;
}


/**
	Calculate centre and scale from supplied bounding box.
	All subsequent vertices must be inside this
*/
bool CVTriangulator::setBound(const BoundingBox& box)
{
    center = box.center();
	
    // scale is max extent 	
    // since we add 25% extra any way might be better to just use box diamter
    scale = box.xMax()-box.xMin();
    if ( (box.yMax()-box.yMin()) > scale )  {
	scale = box.yMax()-box.yMin();
    }
    if ( (box.zMax()-box.zMin()) > scale )  {
	scale = box.zMax()-box.zMin();
    }
    scale*=scaleFactor;		
	
    //scale = box.radius()*2.0;

    center[0] -= scale/2.0f;
    center[1] -= scale/2.0f;
    center[2] -= scale/2.0f;

    if (scale<=0.0 ) {
	return false;
    } 
    return true;
}
/** 
	Can add multiple sets of data. If no bounding box is set it will be calculated from the first set - all extra data must be within this box
*/

bool CVTriangulator::addData(vector<cv::Point3f> vertices,
			     vector<cv::Point3f> normals,
			     vector<float> conf)
{
    if ( vertices.size()==0 ) {
	return false;
    }

    if ( 0.0==scale ) {
	// don't have a bounding box yet - so assume this data is it
	// is there an easy way to do getBound() on a Vec3Array?
	BoundingBox bbox;
	//for(osg::Vec3Array::const_iterator itr=vertices->begin();itr!=vertices->end();itr++) {
	//	bbox.expandBy(*itr);
	//}
	for (int ii=0;ii<(int)vertices.size();ii++) { bbox.expandBy(vertices.at(ii)); }

	if ( !setBound(bbox) ) {
	    return false;
	}
    }

    int splatDepth=maxDepth-2;
    if(splatDepth<0){splatDepth=0;}

    TreeOctNode* temp;



    ////osg::notify(osg::INFO) <<"Setting sample weights\n";
    // update weights - can weight triangulation for each point if you had some point quality value.
    Real weight=1.0f;
    bool use_confidence = false;
    if(conf.size()!=0) 
    {
	// if confidence vector not empty, it has to have the 
	// same size as vertices
//	std::cout << "Using Confidence" << std::endl;
	assert(vertices.size()==conf.size());
	use_confidence = true;
    } else {
	std::cout << "Weighting all vertices equally " << conf.size() 
		  << std::endl;
    }
    Point3D<Real> myCenter;
    Point3D<Real> point;
    
    int n_vertex_calls_=0;

    for (unsigned int ii=0;ii<vertices.size();ii++) {
	point.coords[0] = (vertices.at(ii).x - center[0])/scale;
	point.coords[1] = (vertices.at(ii).y - center[1])/scale;
	point.coords[2] = (vertices.at(ii).z - center[2])/scale;
	
	if(use_confidence)
	{
	    weight = conf.at(ii);
//	    std::cout << weight << std::endl;
	}
	temp=&tree->tree;
	int d=0;
	Real myWidth=1.0;
	myCenter.coords[0]=myCenter.coords[1]=myCenter.coords[2]=0.5;
	for(unsigned int i=0;i<3;i++)
	{
	    if(point.coords[i]<myCenter.coords[i]-myWidth/2 
	       || point.coords[i]>myCenter.coords[i]+myWidth/2)
	    {
		break;
	    }
	}
	
	while(d<splatDepth) {
	    tree->NonLinearUpdateWeightContribution(temp,point,weight);
	    n_vertex_calls_++;
	    if(!temp->children) {	temp->initChildren(); }
	    
	    int cIndex=TreeOctNode::CornerIndex(myCenter,point);
	    temp=&temp->children[cIndex];
	    myWidth/=2;
	    if(cIndex&1){myCenter.coords[0]+=myWidth/2;}
	    else		{myCenter.coords[0]-=myWidth/2;}
	    if(cIndex&2){myCenter.coords[1]+=myWidth/2;}
	    else		{myCenter.coords[1]-=myWidth/2;}
	    if(cIndex&4){myCenter.coords[2]+=myWidth/2;}
	    else		{myCenter.coords[2]-=myWidth/2;}
	    d++;
	}
	tree->NonLinearUpdateWeightContribution(temp,point,weight);
	n_vertex_calls_++;
    }

    std::cout <<"Added vertices "<< n_vertex_calls_ << std::endl;

    //osg::notify(osg::INFO) << "Adding Points and Normals\n";

    Point3D<Real> normal;
    weight = 1.0f;
    // normals are rescaled depending on depth of octree
    float length=float(2<<maxDepth);
/*    if (normals.size()) {
	if (normals.size() < vertices.size() ) {
	    // Assume bind overall and use single value
	    normal.coords[0] = normals.front().x*length;
	    normal.coords[1] = normals.front().y*length;
	    normal.coords[2] = normals.front().z*length;

	}


    } else {
	// assume a flat surface and use up as the normal
	normal.coords[0] = 0.0;
	normal.coords[1] = 0.0;
	normal.coords[2] = length;
    }
*/
    
    int n_normals_calls_ = 0;
    
    for (unsigned int ii=0;ii<vertices.size();ii++) {
	point.coords[0] = (vertices.at(ii).x - center[0])/scale;
	point.coords[1] = (vertices.at(ii).y - center[1])/scale;
	point.coords[2] = (vertices.at(ii).z - center[2])/scale;

	if (normals.size()) {
	    if(use_confidence)
	    {
		weight = conf.at(ii);
	    }
	    normal.coords[0] = normals.at(ii).x*length*weight;
	    normal.coords[1] = normals.at(ii).y*length*weight;
	    normal.coords[2] = normals.at(ii).z*length*weight;
	}
	/*
	printf("%f %f %f  --- %f %f %f --- %f %f %f %f\n",
	       point.coords[0],point.coords[1], point.coords[2],
	       normal.coords[0],normal.coords[1], normal.coords[2],
	       normals.at(ii).x,
	       normals.at(ii).y,
	       normals.at(ii).z,length
	    );
	*/
	myCenter.coords[0]=myCenter.coords[1]=myCenter.coords[2]=0.5;
	float myWidth=1.0;
	for(unsigned int i=0;i<3;i++)
	{
	    if(point.coords[i]<myCenter.coords[i]-myWidth/2 
	       || point.coords[i]>myCenter.coords[i]+myWidth/2)
	    {
		break;
	    }
	}

	tree->NonLinearSplatOrientedPoint(point,
					  normal,
					  splatDepth,
					  samplesPerNode,
					  1,
					  maxDepth);
	n_normals_calls_++;
    }
    

    return vertices.size();
}

struct Normal{
    cv::Vec3f triNormal;
    void normal(cv::Vec3f v1, cv::Vec3f v2, cv::Vec3f v3)
	{
	    cv::Vec3f u,v;

	    // right hand system, CCW triangle
	    u = v2 - v1;
	    v = v3 - v1;
	    triNormal = u.cross(v);
	    triNormal = cv::norm(triNormal);
	}
};/** Start triangulation. (TODO - need to split this up or pass a callback so can update a progress bar)*/
bool CVTriangulator::triangulate(bool get_largest)
{
	
    // this is basically just the steps from 
    // MultiGridOctest.cpp main() in the original source distribution
  
    //osg::notify(osg::INFO) << "# Tree Clipped \n";
    tree->ClipTree();

    //osg::notify(osg::INFO) << "# Finalized 1 \n";
    tree->finalize1(refine);

    //osg::notify(osg::INFO) << "# Laplacian Weights Set \n";
    tree->SetLaplacianWeights();

    //osg::notify(osg::INFO) << "# Finalized 2 \n";
    tree->finalize2(refine);

    //osg::notify(osg::INFO) << "# Linear System Solved \n";
    tree->LaplacianMatrixIteration(solverDivide);
    
    //osg::notify(osg::INFO) << "# Build Triangles \n";
    mesh = new CoredVectorMeshData;
  
    Real isoValue=tree->GetIsoValue();

    tree->GetMCIsoTriangles(isoValue,mesh);

    //osg::notify(osg::INFO) << "# Got Triangles \n";
  
    // Convert into osg vertex/triangle data
    if (!mesh) {
	return false;
    }

    int npoints = mesh->inCorePoints.size();
    cout << "Number of points " << npoints<<endl;

    if (!npoints) {
	return false;
    }

    // regridded points
    //_vertices = new osg::Vec3Array;
    //	_vertices->reserve(npoints);
    int ntri = mesh->triangleCount();
    TriangleIndex tIndex;
    int inCoreFlag;
	
    if(get_largest){
	Clean cleanMesh;
	Vector3 p;
	for (int ii=0;ii<npoints;ii++){
	    p[0] = mesh->inCorePoints[ii].coords[0];
	    p[1] = mesh->inCorePoints[ii].coords[1];
	    p[2] = mesh->inCorePoints[ii].coords[2];
	    
	    cleanMesh.addVertex(p);
	}
	  
	if ( !ntri ) {
	    return false;
	}
	cout << "Number of Tri " << ntri<<endl;
	for (int ii=0;ii<ntri;ii++) {
	    mesh->nextTriangle(tIndex,inCoreFlag);
	    if (inCoreFlag) {
		vector<int> verts;
	      
		verts.push_back(tIndex.idx[0]);
		verts.push_back(tIndex.idx[1]);
		verts.push_back(tIndex.idx[2]);
		cleanMesh.addFace(verts);
	    }
	}
	cleanMesh.finishMesh();
	cleanMesh.computeMeshInfo();


	mesh_info_.boundary_loops_ = cleanMesh.numBoundaryLoops;
	mesh_info_.boundary_vertices_ = cleanMesh.numBoundaryVertices;
	mesh_info_.connected_components_ = cleanMesh.numConnectedComponents;
	mesh_info_.manifold_ = cleanMesh.checkManifold();
	    
	cv::Vec3f point;
	for (int ii=0;ii<cleanMesh.numVertices();ii++){
	    point[0] = cleanMesh.vertexAt(ii)->p[0];
	    point[1] = cleanMesh.vertexAt(ii)->p[1];
	    point[2] = cleanMesh.vertexAt(ii)->p[2];
	    
	    
	    _vertices.push_back((point*scale)+center);
	}
  
	cout << "Points ="<<npoints<<"\n";
	if(get_largest)
	    ntri=cleanMesh.cc_face[0].size();
	else
	    ntri=cleanMesh.numFaces();
	//_triangles = new osg::DrawElementsUInt(GL_TRIANGLES,ntri*3);
	  
	for( size_t i = 0; i < _vertices.size(); ++i )
	{
	    _normals.push_back( cv::Vec3f( 0, 0, 0 ) );
	}
	for (int ii=0;ii<ntri;ii++) {
	    vector<int> idx;
	    Face *fPtr;
	    if(get_largest)
		fPtr=cleanMesh.cc_face[0][ii];	
	    else
		fPtr= cleanMesh.faceAt(ii);

	      
	    Face::EdgeAroundIterator around=fPtr->iterator();
	

	    for ( ; !around.end(); around++)
		idx.push_back(around.vertex()->ID );

	
	    _triangles.push_back(idx[0]);
	    _triangles.push_back(idx[1]);
	    _triangles.push_back(idx[2]);

	}
    }else{
	cv::Vec3f p;
    
	for (int ii=0;ii<npoints;ii++){
	    p[0] = mesh->inCorePoints[ii].coords[0];
	    p[1] = mesh->inCorePoints[ii].coords[1];
	    p[2] = mesh->inCorePoints[ii].coords[2];
	    _vertices.push_back((p*scale)+center);
	}
	for( size_t i = 0; i < _vertices.size(); ++i )
	{
	    _normals.push_back( cv::Vec3f( 0, 0, 0 ) );
	}
	for (int ii=0;ii<ntri;ii++) {
	    mesh->nextTriangle(tIndex,inCoreFlag);
	    if (inCoreFlag) {
		_triangles.push_back(tIndex.idx[0]);
		_triangles.push_back(tIndex.idx[1]);
		_triangles.push_back(tIndex.idx[2]);
	    }
	}

    }
    return true;
}

MeshInfo CVTriangulator::getMeshInfo()
{
    return mesh_info_;
}

