/** \file Mesh.cpp Definitions for the Mesh class.
    \author Liliya Kharevych
    \date March 2006
*/

#include "Clean.h"
#include <iostream>
#include <fstream>
//#include "util.h"
#include <stack>
#include <iomanip>
#include <algorithm>

using namespace std;
static void check_error (bool expr, const std::string & message) {
   if (expr) {
      std::cerr << "Error: " << message << std::endl;
      throw 1;
      //      exit(1);
   }
}
Clean::Clean(void)
{
   numBoundaryVertices = 0;
}

Clean::~Clean(void)
{
   clear();
}

void Clean::clear() {
   VertexIterator vIter = vertexIterator();
   FaceIterator fIter = faceIterator();
   HalfEdgeIterator eIter = halfEdgeIterator();

   while (!vIter.end()) {
      delete vIter.vertex();
      vIter++;
   }
   while (!fIter.end()) {
      delete fIter.face();
      fIter++;
   }
   while (!eIter.end()) {
      delete eIter.half_edge();
      eIter++;
   }
}

Vertex * Clean::addVertex(const Vector3 & _p) {
   Vertex * v = new Vertex();
   v->p = _p;
   v->ID = (int)vertices.size();
   vertices.push_back(v);
   return v;
}


Face * Clean::addFace(std::vector<int> faceVerts) {
   Face * f = new Face();
   f->ID = (int)faces.size();

   CleanEdge * firstEdge = NULL;
   CleanEdge * last = NULL;
   CleanEdge * current = NULL;

   unsigned int i;

   for (i = 0; i < faceVerts.size()-1; i++) {
      current = addEdge(faceVerts[i], faceVerts[i+1]);

      check_error (!current, "Problem while parsing mesh faces.");
      
      current->face = f;

      if (last)
         last->next = current;
      else
         firstEdge = current;
      last = current;
   }

   current = addEdge (faceVerts[i], faceVerts[0]);
   check_error (!current, "Problem while parsing mesh faces.");

   current->face = f;

   last->next = current;
   current->next = firstEdge;

   f->edge = firstEdge;
   faces.push_back(f);

   return f;
}

CleanEdge * Clean::addEdge (int i, int j) {
   CleanEdge eTmp;
   eTmp.vertex = vertices[i];

   CleanEdge eTmpPair;
   eTmpPair.vertex = vertices[j];
   eTmp.pair = &eTmpPair;

   Clean::EdgeSet::iterator eIn = edges.find(&eTmp);
   CleanEdge * e;

   if (eIn != edges.end()){
      e = *eIn;
      if (e->face != NULL)
	return NULL;
   }
   else {
      e = new CleanEdge();
      CleanEdge * pair = new CleanEdge();

      e->vertex = vertices[i];
      pair->vertex = vertices[j];

      pair->pair = e;
      e->pair = pair;

      edges.insert(e);
      edges.insert(pair);

      pair->vertex->edge = pair;
   }
   return e;
}


/** Compute edge lengths based on vertex positions */
void Clean::computeLengths() {
   for (Clean::EdgeIterator eIter = edgeIterator(); !eIter.end(); eIter++) {
      CleanEdge * e = eIter.edge();
      e->length = (e->vertex->p - e->next->vertex->p).abs();
      e->pair->length = e->length;
   }
}

/** Called after the mesh is created to link boundary edges */
void Clean::linkBoundary() {
   HalfEdgeIterator hIter = halfEdgeIterator();

   for (; !hIter.end(); hIter++) {
      if (!hIter.half_edge()->face)
         hIter.half_edge()->vertex->edge = hIter.half_edge();
   }

   VertexIterator vIter = vertexIterator();
   CleanEdge *next, *beg;
   while (!vIter.end()) {
      if (vIter.vertex()->isBoundary()) {
         beg = vIter.vertex()->edge;
         next = beg;
         while (beg->pair->face)
            beg = beg->pair->next;
         beg = beg->pair;
         beg->next = next;
         numBoundaryVertices++;
      }
      vIter++;
   }

}

/** Computes information about the mesh:

Number of boundary loops,
Number of connected components,
Genus of the mesh.

Only one connected compoment with 0 or 1 boundary loops can be parameterize.
Singularities should be assigned in such a way that Gauss-Bonet theorem is satisfied.
*/
bool lessLength (const std::vector<Face *> &v1, const std::vector<Face *> &v2)

//bool lessLength (const  std::pair<int,Face *> &v1, const  std::pair<int,Face *> &v2)
{
  return v1.size() > v2.size();
  //  return v1.first > v2.first;
}

void Clean::computeMeshInfo() {
   cout << "Topological information about the mesh:" << endl;
   // Number of boundary loops
   Clean::HalfEdgeIterator hIter = halfEdgeIterator();
   for (; !hIter.end(); hIter++) {
      hIter.half_edge()->check = false;
   }
   numBoundaryLoops = 0;
   for (hIter.reset(); !hIter.end(); hIter++) {
      CleanEdge * e = hIter.half_edge();
      if (e->face)
         e->check = true;
      else if (!e->check) {
         CleanEdge * beg = NULL;
         while (e != beg) {
            if (!beg) beg = e;
            check_error(!e, "Building the mesh failed, probem with input format.");
            e->check = true;
            e = e->next;
         }
         numBoundaryLoops++;
      }
   }
   cout << "Mesh has " << numBoundaryLoops << " boundary loops." << endl;
   // Number of connected components
   numConnectedComponents = 0;
   Clean::FaceIterator fIter = faceIterator();
   for (; !fIter.end(); fIter++) {
      fIter.face()->check = false;
   }
   stack<CleanEdge *> toVisit;
   cc_face.clear();
   
   for (fIter.reset(); !fIter.end(); fIter++) {
      if (!fIter.face()->check) {
       	cc_face.push_back(std::vector<Face *>(1,fIter.face()));
	fIter.face()->check = true;
	// numConnectedComponents++;
  //	 cc_face.resize(numConnectedComponents);
         toVisit.push(fIter.face()->edge);
         while (!toVisit.empty()) {
            Face * fIn = toVisit.top()->face;
	    cc_face.back().push_back(fIn);
            toVisit.pop();
            Face::EdgeAroundIterator iter = fIn->iterator();
            for (; !iter.end(); iter++)
	      if (iter.edge()->pair->face && !iter.edge()->pair->face->check){
                  toVisit.push(iter.edge()->pair);
		  iter.edge()->pair->face->check = true;

	      }
	 }
	 numConnectedComponents++;

      }
   
   }
   /*if(int(cc_face.size())!=numConnectedComponents){
     printf("ASDASDASDAS %d ==== %d\n",cc_face.size(),numConnectedComponents);
   }*/

   cout << "Mesh has " << numConnectedComponents << " connected components." << endl;
   sort (cc_face.begin(), cc_face.end(),           // range
     lessLength);                          // criterion
// for(int i=0; i <cc_face.size(); i++)
  // printf("Size %d\n",cc_face[i].size());
   // Genus number
 /* check_error(numConnectedComponents == 0, "The mesh read is empty.");
   numGenus =
      (1 - (numVertices() - numEdges() + numFaces() + numBoundaryLoops ) / 2) / numConnectedComponents;
      cout << "Mesh is genus " << numGenus << "." << endl;*/
}

/** Check if all the vertices in the mesh have at least on edge coming out of them */
bool Clean::checkVertexConection() {
   Clean::FaceIterator fIter = faceIterator();
   Clean::VertexIterator vIter = vertexIterator();
   bool conectedVert = true;

   for (;!vIter.end(); vIter++)
      vIter.vertex()->check = false;

   for (fIter.reset(); !fIter.end(); fIter++) {
      Face::EdgeAroundIterator around = fIter.face()->iterator();
      for (;!around.end();around++)
         around.vertex()->check = true;
   }
   for (vIter.reset(); !vIter.end(); vIter++) {
      if (!vIter.vertex()->check) {
         cerr << "Vertex " << vIter.vertex()->ID << " is not connected." << endl;
         conectedVert = false;
      }
   }

   return conectedVert;
}

/** Manifoldness check: only one disk should be adjusted on any vertex */
bool Clean::checkManifold() {
   Clean::HalfEdgeIterator eIter = halfEdgeIterator();
   Clean::VertexIterator vIter = vertexIterator();
   bool manifold = true;

   for (;!eIter.end(); eIter++)
      eIter.half_edge()->check = false;

   for (vIter.reset(); !vIter.end(); vIter++) {
      Vertex::EdgeAroundIterator around = vIter.vertex()->iterator();
      for (;!around.end();around++)
         around.edge_out()->check = true;
   }

   for (eIter.reset(); !eIter.end(); eIter++) {
      if (!eIter.half_edge()->check) {
         cerr << "Mesh is not manifold - more then one disk at vertex "
            << eIter.half_edge()->vertex->ID << endl;
         manifold = false;
         break;
      }
   }

   return manifold;
}

/** Compute initial alpha angles in the mesh.

This function assumes that the mesh is triangular.
Alpha angels are stored at each edge and correspond
to an angle opposite to this edge.
*/
void Clean::computeInitAngles() {
   Clean::HalfEdgeIterator eIter = halfEdgeIterator();
   for (; !eIter.end(); eIter++) {
      CleanEdge * e = eIter.half_edge();
      if (e->face) {
         double l1 = e->length;
         double l2 = e->next->length;
         double l3 = e->next->next->length;
         e->alphaOposite = acos((l2*l2 + l3*l3 - l1*l1)/(2.0*l2*l3));
      }
      else {
         e->alphaOposite = 0;
      }
   }
   for (eIter.reset(); !eIter.end(); eIter++) {
      CleanEdge * e = eIter.half_edge();
      e->setTheta(M_PI - e->alphaOposite - e->pair->alphaOposite);
   }
}

/** Loads mesh from obj file

Standard format for OBJ file is

v double double double

v double double double

f int int int

f int int int

Files with vt tags also can be parsed
*/
void Clean::readOBJ(const char * obj_file) {
   string front;
   string v = "v", vt = "vt", f = "f";
   Vector3 vert;
   vector<int> verts;
   vector<Vector3> uvVec;
   vector<int> uvs;
   char etc;
   int id;

   ifstream in(obj_file);

   check_error(!in, "Cannot find input obj file.");

   bool hasUV = false;

   while(!in.eof() || in.peek() != EOF) {
      in >> ws;
      if (in.eof() || in.peek() == EOF)
         break;
      if (in.peek() == '#') {
         in.ignore(300, '\n');
      }
      else {
         in >> front;
         if (front == v) {
            in >> vert.x() >> vert.y() >> vert.z();
            addVertex(vert);
         }
         else if (front == vt){
            in >> vert.x() >> vert.y();
            vert.z() = 0;
            uvVec.push_back(vert);
            hasUV = true;
         }
         else if (front == f) {
            verts.clear();
            uvs.clear();
            while (in >> id) {

               check_error(id > numVertices(), "Problem with input OBJ file.");

               verts.push_back(id-1);
               bool vtNow = true;
               if (in.peek() == '/'){
                  in >> etc;
                  if (in.peek() != '/') {
                     in >> id;
                     //check_warn(id > numVertices(), "Texture coordinate index is greater then number of vertices.");
                     if (id < numVertices() && hasUV) {
                        uvs.push_back(id-1);
                        vtNow = false;
                     }
                  }
               }
               if (in.peek() == '/'){
                  int tmp;
                  in >> etc;
                  in >> tmp;
               }
               if (hasUV && vtNow) {
                  uvs.push_back(id-1);
               }
            }
            in.clear(in.rdstate() & ~ios::failbit);
            Face * f = addFace(verts);

            if (hasUV && uvs.size() != 0){
               int k = 0;
               for (Face::EdgeAroundIterator e = f->iterator(); !e.end(); e++, k++)
                  e.vertex()->uv = uvVec[uvs[k]];
            }
         }
         else {
            string line;
            getline(in, line);
            cout << "Warning, line: " << line << " ignored." << endl;
         }
      }
   }

   in.close();

   // Finnish building the mesh, should be called after each parse.
   finishMesh();
}
