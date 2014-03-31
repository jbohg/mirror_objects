/** \file Vector3.h Declarations and definitions for the Vector3 class.
    \author Liliya Kharevych
    \date March 2006
*/

#ifndef VECTOR3_CM
#define VECTOR3_CM

#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <iostream>
#define _USE_MATH_DEFINES
#include <math.h>

/**
Dimension of the vector.

\warning Most of the functions can be used
only with 3 dimensional vectors
*/
const int vDim = 3;

/** This class defines operations for 3 dimensional vectors
*/

class Vector3
{

private:
   double vec[vDim];

   void copy (const Vector3 & o){
      for (int i = 0; i < vDim; i++)
	 vec[i] = o.vec[i];
   }

   const Vector3& operator*=(const Vector3& rhs){
      for (int i = 0; i < vDim; i++)
	 vec[i] = vec[i]*rhs.vec[i];		
      return *this;
   }

public:


   /** Returns unit vector in x direction          */
   static Vector3 e1() {return Vector3(1, 0, 0);}
   /** Returns unit vector in y direction          */
   static Vector3 e2() {return Vector3(0, 1, 0);}
   /** Returns unit vector in z direction          */
   static Vector3 e3() {return Vector3(0, 0, 1);}
   /** Returns 3x1 vector of all ones              */
   static Vector3 ones() {return Vector3(1, 1, 1);}

   /** Creates empty vector of zeros       */
   Vector3(void){
      for (int i = 0; i < vDim; i++)
	 vec[i] = 0;
   }

   /** Creates new vector
   \param x New x component of the vector
   \param y New y component of the vector
   */
   Vector3(double x, double y){
      vec[0] = x; vec[1] = y; vec[2] = 0;
   }

   Vector3(double x, double y, double z){
      vec[0] = x; vec[1] = y; vec[2] = z;
   }

   Vector3(const Vector3 & o){
      copy(o);
   }

   /** Equal operator
   \return Vectors are equal when each component differs less then by 1.e-14
   */	
   bool operator==(const Vector3& rhs) const{
      return (fabs(vec[0] - rhs.vec[0]) < 1.e-14 && 
	 fabs(vec[1] - rhs.vec[1]) <  1.e-14 && 
	 fabs(vec[2] - rhs.vec[2]) <  1.e-14);
   }

   bool operator!=(const Vector3& rhs) const{
      return (!(*this == rhs));
   }

   const Vector3& operator=(const Vector3& rhs){
      if (this != &rhs)
	 copy(rhs);
      return *this;
   }

   const Vector3& operator+=(const Vector3& rhs){
      for (int i = 0; i < vDim; i++)
	 vec[i] = vec[i]+rhs.vec[i];		
      return *this;
   }

   const Vector3& operator-=(const Vector3& rhs){
      for (int i = 0; i < vDim; i++)
	 vec[i] = vec[i]-rhs.vec[i];		
      return *this;
   }


   const Vector3& operator*=(const double& rhs){
      for (int i = 0; i < vDim; i++)
	 vec[i] = vec[i]*rhs;		
      return *this;
   }

   const Vector3& operator/=(const double& rhs){
      for (int i = 0; i < vDim; i++)
	 vec[i] = vec[i]/rhs;		
      return *this;
   }

   // Arithmetic
   Vector3 operator+(const Vector3& rhs) const {
      Vector3 tmp(*this);
      tmp += rhs;
      return tmp;
   }


   Vector3 operator-(const Vector3& rhs) const {
      Vector3 tmp(*this);
      tmp -= rhs;
      return tmp;
   }	

   /**
   Dot product of vectors
   */
   double dot(const Vector3& rhs) const {
      Vector3 tmp(*this);
      tmp *= rhs;
      double sum = 0;
      for (int i = 0; i < vDim; i++)
	 sum += tmp.vec[i];
      return sum;
   }

   Vector3 operator*(const double& rhs) const {
      Vector3 tmp(*this);
      tmp *= rhs;
      return tmp;
   }

   Vector3 operator/(const double& rhs) const {
      Vector3 tmp(*this);
      tmp /= rhs;
      return tmp;
   }


   /**
   Access operator, first element has index zero.
   */
   double & operator[] (int i){
      return vec[i];
   }

   /**
   Cross product of 3x1 vectors
   */
   Vector3 crossProduct(const Vector3& rhs) const{
      Vector3 tmp;
      tmp[0] = vec[1]*rhs[2] - vec[2]*rhs[1];
      tmp[1] = vec[2]*rhs[0] - vec[0]*rhs[2];
      tmp[2] = vec[0]*rhs[1] - vec[1]*rhs[0];
      return tmp;
   }

   /**
   Access operator, first element has index zero.
   */
   double operator[] (int i) const{
      if (i >= vDim || i < 0){
	 std::cerr << "Out of bounds vector accessing: [" << i << "]" << std::endl;
	 return 0;
      }
      else
	 return vec[i];
   }

   double x() const {return vec[0];}
   double & x() {return vec[0];}
   double y() const {return vec[1];}
   double & y() {return vec[1];}
   double z() const {return vec[2];}
   double & z() {return vec[2];}

   /**
   Absolute value of the vector
   */
   double abs () const {
      double sum = 0;
      for (int i = 0; i < vDim; i++)
	 sum += vec[i]*vec[i];
      return sqrt(sum);
   }

   /**
   Returns normalized vector
   \return nomalized vector in same direction
   */
   Vector3 norm() const {
      Vector3 norm;
      double length = abs();
      for (int i = 0; i < vDim; i++)
	 norm[i] = vec[i]/length;
      return norm;
   }

   /**
   Normalize current vector
   */
   void normalize(){
      double length = abs();
      for (int i = 0; i < vDim; i++)
	 vec[i] = vec[i]/length;
   }

   /**
   Negate each element of the vector
   */
   void negate () {
      vec[0] = -vec[0];
      vec[1] = -vec[1];
      vec[2] = -vec[2];
   }

   // Input/Output
   void print(std::ostream& out, char * name) const{ 
      for (int i = 0; i < vDim; i++)
	 out << name << "(" << (i+1) << ")= " << vec[i] << "; ";
      out << std::endl;
   } 

   friend std::ostream& operator<<(std::ostream& out,const Vector3 & m) { 
      out << m.vec[0];
      for (int i = 1; i < vDim; i++)
	 out << "  " << m.vec[i];
      return out; 
   } 


   friend std::istream& operator>>(std::istream& in, Vector3 & m) { 
      for (int i = 0; i < vDim; i++)
	 in >> m.vec[i];
      return in; 
   } 
};

#endif

