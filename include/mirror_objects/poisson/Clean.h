/** \file Mesh.h Declarations for the Mesh class.
    \author Liliya Kharevych
    \date March 2006
*/

#ifndef MESH_CM
#define MESH_CM
#include <vector>
#include <set>
#include <iostream>
#include <stdlib.h>

class Vertex;

#include "V3.h"
class CleanEdge;
class Face;
class CleanEdge
{
private:
	/** theta angles */
   double th;
   double cTheta;
   double sTheta;
   double cl2TS;
   double tanHTS;

public:

   // pointers for mesh structure
   CleanEdge * next;
   CleanEdge * pair;
   Vertex * vertex;
   Face * face;

   /** Alpha angle oposite to the edge (different for pair edges). */
   double alphaOposite;

   /** Check for various iterations*/
   bool check;

   /**Length of the edge (same for pair edges). */
   double length;

   /**Used for angle optimization */
   int ID;

   /**If the mesh is cut into multiple patches, 
   this stores the patch ID that edge belongs to. */
   int patchID;


   CleanEdge(void) : next(NULL), pair(NULL), vertex(NULL), face(NULL),
      length(0)
   {
      patchID = 0;
      th = 0;
      alphaOposite = 0;
   }

   // Assignment (copy only geometrical information)
   void assignData(const CleanEdge& rhs){
      if (this != &rhs) {
	 alphaOposite = rhs.alphaOposite;
	 length = rhs.length; 
	 th = rhs.th; 
	 cTheta = rhs.cTheta; 
	 sTheta = rhs.sTheta; 
	 cl2TS = rhs.cl2TS; 
	 tanHTS = rhs.tanHTS;
      }
   }

   bool isBoundary() const {return !(face && pair->face);}
   Vertex * oppositeVertex() {return next->next->vertex;}

   const double & tanHalfTStar() const {return tanHTS;}
   const double & sinTheta() const {return sTheta;}
   const double & cosTheta() const {return cTheta;}

   const double & theta() const {return th;}
    void setTheta(const double & _th) {
      th = _th;
      cTheta = cos(th);
      sTheta = sin(th);
      tanHTS = tan(0.5*(M_PI - th));

      }

};
/** Vertex for the mesh datastructure.
*/
class Vertex
{
private:
   const Vertex& operator=(const Vertex& rhs){ return *this;}
public:

   // pointers for mesh structure
   CleanEdge * edge;

   // geometry data
   Vector3 p;
   Vector3 uv;

   // Cone angle data (multiple of pi is stored)
   double min_cone_angle;
   double max_cone_angle;
   bool constrained;
   double cone_angle;

   // distinct id
   int ID;
   int patchID;

   // to check various iterations
   bool check;

   Vertex (const Vector3 & _pos = Vector3(0,0,0)): 
   edge(NULL), p(_pos), uv(0,0,0)
   {
      cone_angle = min_cone_angle = max_cone_angle = 2;
      constrained = false;
      patchID = 0;
   }

   // Assignment (copy only geometrical information)
   void assignData(const Vertex& rhs){
      if (this != &rhs) {
	 p = rhs.p;
	 uv = rhs.uv;
	 cone_angle = rhs.cone_angle;
	 min_cone_angle = rhs.min_cone_angle;
	 max_cone_angle = rhs.max_cone_angle;
      }
   }


   bool isBoundary() {
      return (edge && !edge->face);
   }

  /** The iterator that visits edges or vertices in one ring of the current face in order. */
  class EdgeAroundIterator {
   private:
      CleanEdge * endI;
      CleanEdge * run;

   public:
      EdgeAroundIterator(CleanEdge * e) {
	 endI = NULL;
	 run = e;
      }
      EdgeAroundIterator& operator++( void ){
	 if (!endI) endI = run;
	 run = run->pair->next;
	 return *this;
      }
      EdgeAroundIterator operator++( int ){
	 EdgeAroundIterator r = *this; ++*this; return r;
      }

      CleanEdge * edge_out( void ) const { return run; }
      CleanEdge * & edge_out( void )     { return run; }

      Vertex * vertex( void ) const { return run->vertex; }
      Vertex * & vertex( void )     { return run->vertex; }

      bool end(void) { return endI == run;}
   };

   EdgeAroundIterator iterator() {return EdgeAroundIterator(edge);}
   EdgeAroundIterator iterator(CleanEdge * eFrom) {return EdgeAroundIterator(eFrom);}

   int getValence() {
      Vertex::EdgeAroundIterator iter = iterator();
      int res = 0;
      for (; !iter.end(); iter++)
	 res++;
      return res;
   }
};

/** Face for the mesh datastructure.

Faces are always assumed to be triangles. Although circle pattern algorithm can be extended to 
polygonal meshes, we currently limit it to triangular meshes only. In addition to standard 
data members, radius of the circum circle is stored for each edge.
*/
class Face
{
public:

   // pointers for mesh structure
   CleanEdge * edge;

   // distinct id
   int ID;
   int patchID;

   // to check various iterations
   bool check;

   /** Circum circle radius of this triangle (solution of circle pattern functional). */
   double r;

   Face(void): edge(NULL), ID(-1) 
   { 
      patchID = 0;
   }

   /** The iterator that visits edges, vertices, or faces around the current face in order. */
   class EdgeAroundIterator {
   private:
      CleanEdge * endI;
      CleanEdge * run;
      CleanEdge * _e;
   public:

      EdgeAroundIterator(CleanEdge * e) {
	 endI = NULL;
	 run = e;
	 _e = e;
      }

      void reset() {
	 endI = NULL;
	 run = _e;
      }

      EdgeAroundIterator& operator++( void ){
	 if (!endI) endI = run;
	 run = run->next;
	 return *this;
      }
      EdgeAroundIterator operator++( int ){
	 EdgeAroundIterator r = *this; ++*this; return r;
      }

      CleanEdge * edge( void ) const { return run; }
      CleanEdge * & edge( void )     { return run; }

      Vertex * vertex( void ) const { return run->vertex; }
      Vertex * & vertex( void )     { return run->vertex; }

      Face * face( void ) const { return run->pair->face; }
      Face * & face( void )     { return run->pair->face; }

      bool end(void) { return endI == run;}
   };

   EdgeAroundIterator iterator() {return EdgeAroundIterator(edge);}
   EdgeAroundIterator iterator(CleanEdge * myE) {return EdgeAroundIterator(myE);}

};
/** Half edge for the mesh datastructure.

In addition to the usual members which edge for the half-edge datastructure has 
(such as pointers to the next and pair edges, pointer to the vertex at the start 
of the edge and pointer to the touching face); 
this edge class has some extra data for circle pattern algorithm. 
The main data is a length of an edge and a theta angle associated with an edge. 
We also store an alpha angle associated with each edge (aplhaOposite). 
This angle is formed by another two edges of the triangle which touches the edge. 
For boundary edges alphaOposite are equal to 0.
Various functions of theta angle are precomputed and stored: 
cos(theta), sin(theta), tan(0.5*(M_PI - theta)),
Cl2(2.0*(M_PI - theta)).


*/




/**
Implementation of the half edge datastructure.
*/

class Clean
{
private:
   /** Comparison functions for edge sets

   Lexicographical order by indices of start end end point of the edge
   */
   struct edge_comp {
      edge_comp() {}
      bool operator()( const CleanEdge *e1, const CleanEdge *e2 ) const {
         int b1 = e1->vertex->ID, t1 = e1->pair->vertex->ID;
         int b2 = e2->vertex->ID, t2 = e2->pair->vertex->ID;

         int min1, min2, max1, max2;
         min1 = std::min(b1, t1); min2 = std::min(b2, t2);
         max1 = std::max(b1, t1); max2 = std::max(b2, t2);
         if (min1 == min2 && max1 == max2){
            if (b1 < b2) return true;
            return (b1 == b2 && t1 < t2);
         }
         if (min1 < min2)     return true;
         return (min1 == min2 && max1 < max2);
      }
   };


   std::vector<Vertex *> vertices;
   std::vector<Face *> faces;
   std::multiset<CleanEdge*, Clean::edge_comp> edges;
  std::vector<std::vector<Vertex *> >cc_verts;


public:
std::vector<std::vector<Face *> >cc_face;
  //std::vector< std::pair<int,Face *> > CCV;
   /** Number of vertices (or also edges) at the boundary of the mesh   */
   int numBoundaryVertices;

   /** Number of boundary loops in the mesh */
   int numBoundaryLoops;
   /** Number of connected components */
   int numConnectedComponents;

   /** Genus of the mesh */
   int numGenus;

   typedef std::multiset<CleanEdge*, Clean::edge_comp>  EdgeSet;
   typedef std::vector<Face *> FaceSet;
   typedef std::vector<Vertex *> VertexSet;

   Clean(void);
   ~Clean(void);
   void clear();

   Vertex * addVertex(const Vector3 & p);
   Face * addFace(std::vector<int> faceVerts);
   CleanEdge * addEdge(int i, int j);
   CleanEdge * addEdge(CleanEdge * e) {
      edges.insert(e);
      return e;
   }

   bool cutAlongEdge(CleanEdge * forward);

   inline Vertex * vertexAt (int i) {return vertices[i];}
   inline Face * faceAt (int i) {return faces[i];}

   inline int numFaces()     { return (int)faces.size();    }
   inline int numVertices()  { return (int)vertices.size(); }
   inline int numEdges()     { return (int)edges.size()/2;  }
   inline int numBoundary()  { return numBoundaryVertices;  }

   void linkBoundary();
   bool checkManifold();
   bool checkVertexConection();
   void checkGaussianCurvature();

   /**
   Computes number of connected components, number of boundary loops and genus of the mesh.

   Uses variation of Euler Formula: V - E + F = 2 (c - g)
   Where V - number of vertices.
   E - number of edges.
   F - number of faces.
   c - number of connected components.
   */
   void computeMeshInfo();

   void computeLengths();
   void computeInitAngles();

   void readOBJ(const char * obj_file);
   void readCON(const char * conn_file);
   void readCONE_VERT(const char * vert_file);
   void readCUT_EDGES(const char * edge_file);
   void writeOBJ(const char * obj_file);
   void writeVT(const char * vt_file);
   void writeCON(const char * conn_file);

   void finishMesh() {
      linkBoundary();
      checkManifold();
      checkVertexConection();
      std::cout << "*--------------------*" << std::endl;
      std::cout << "* Faces:    " << numFaces() << std::endl;
      std::cout << "* Edges:    " << numEdges() << std::endl;
      std::cout << "* Vertices: " << numVertices() << std::endl;
      std::cout << "*--------------------*\n" << std::endl;
   }

   class FaceIterator {
   private:
      FaceSet::iterator fIter;
      FaceSet * facesPtr;
   public:
      FaceIterator() {
         facesPtr = NULL;
      }

      FaceIterator(FaceSet * _faces) {
         facesPtr = _faces;
         fIter = _faces->begin();
      }
      FaceIterator& operator++( void ){
         fIter++;
         return *this;
      }
      FaceIterator operator++( int ){
         FaceIterator r = *this; ++*this; return r;
      }
      FaceIterator& operator--( void ){
         fIter--;
         return *this;
      }
      FaceIterator operator--( int ){
         FaceIterator r = *this; --*this; return r;
      }

      Face * face( void ) const { return *fIter; }
      //Face * & face( void )     { return *fIter; }

      void reset() {fIter = facesPtr->begin(); }
      bool end(void) { return fIter == facesPtr->end();
      ;}
   };

   class VertexIterator {
   private:
      VertexSet::iterator vIter;
      VertexSet * verticesPtr;
   public:
      VertexIterator() {
         verticesPtr = NULL;
      }

      VertexIterator(VertexSet * _vertices) {
         vIter = _vertices->begin();
         verticesPtr = _vertices;
      }
      VertexIterator& operator++( void ){
         vIter++;
         return *this;
      }
      VertexIterator operator++( int ){
         VertexIterator r = *this; ++*this; return r;
      }
      VertexIterator& operator--( void ){
         vIter--;
         return *this;
      }
      VertexIterator operator--( int ){
         VertexIterator r = *this; --*this; return r;
      }

      Vertex * vertex( void ) const { return *vIter; }
      //Vertex * & vertex( void )     { return *vIter; }

      void reset() {vIter = verticesPtr->begin();}
      bool end(void) { return vIter == verticesPtr->end();}
   };

   class HalfEdgeIterator {
   private:
      EdgeSet::iterator eIter;
      EdgeSet * edgesPtr;
   public:
      HalfEdgeIterator() {
         edgesPtr = NULL;
      }
      HalfEdgeIterator(EdgeSet * _edges) {
         eIter = _edges->begin();
         edgesPtr = _edges;
      }
      HalfEdgeIterator& operator++( void ){
         eIter++;
         return *this;
      }
      HalfEdgeIterator operator++( int ){
         HalfEdgeIterator r = *this; ++*this; return r;
      }
      HalfEdgeIterator& operator--( void ){
         eIter--;
         return *this;
      }
      HalfEdgeIterator operator--( int ){
         HalfEdgeIterator r = *this; --*this; return r;
      }

      CleanEdge * half_edge( void ) const { return *eIter; }
      //                Edge * & operator*( void )     { return *eIter; }

      void find (CleanEdge * eTmp) {
         eIter = edgesPtr->find(eTmp);
      }

      void reset() {eIter = edgesPtr->begin(); }
      bool end (void) { return eIter == edgesPtr->end();}
   };

   class EdgeIterator {
   private:
      EdgeSet::iterator eIter;
      EdgeSet * edgesPtr;

   public:
      EdgeIterator() {
         edgesPtr = NULL;
      }

      EdgeIterator(EdgeSet * _edges) {
         eIter = _edges->begin();
         edgesPtr = _edges;
      }
      EdgeIterator& operator++( void ){
         eIter++; eIter++;
         return *this;
      }
      EdgeIterator operator++( int ){
         EdgeIterator r = *this; ++*this; return r;
      }
      EdgeIterator& operator--( void ){
         eIter--; eIter--;
         return *this;
      }
      EdgeIterator operator--( int ){
         EdgeIterator r = *this; --*this; return r;
      }

      CleanEdge * edge( void ) const { return *eIter; }
      //                Edge * & operator*( void )     { return *eIter; }

      void reset() {eIter = edgesPtr->begin(); }
      bool end(void) { return eIter == edgesPtr->end();}
   };



   FaceIterator faceIterator() {return FaceIterator(&faces);}
   VertexIterator vertexIterator() {return VertexIterator(&vertices);}
   HalfEdgeIterator halfEdgeIterator() {return HalfEdgeIterator(&edges);}
   EdgeIterator edgeIterator() {return EdgeIterator(&edges);}
};

#endif
