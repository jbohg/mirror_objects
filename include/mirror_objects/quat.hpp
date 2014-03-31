/*********************************************************************
*
*  Copyright (c) 2011, Matthew Johnson-Roberson - KTH Stockholm
*  (mattjr@csc.kth.se)
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
*   * Neither the name of Matthew Johnson-Roberson nor the names of KTH
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

/* This file is adapated from the OpenSceneGraph quaternion class
 * and changed to be dependent on OpenCV data structures.
 */

#ifndef CV_QUAT
#define CV_QUAT 1

#include <opencv2/opencv.hpp>


using cv::Vec4d;
using cv::Vec4f;
using cv::Vec3f;
using cv::Vec3d;

/** A quaternion class. It can be used to represent an orientation in 3D space.*/
class  Quat
{

    public:

        typedef double value_type;

        value_type  _v[4];    // a four-vector

        inline Quat() { _v[0]=0.0; _v[1]=0.0; _v[2]=0.0; _v[3]=1.0; }

        inline Quat( value_type x, value_type y, value_type z, value_type w )
        {
            _v[0]=x;
            _v[1]=y;
            _v[2]=z;
            _v[3]=w;
        }

        inline Quat( const Vec4f& v )
        {
            _v[0]=v[0];
            _v[1]=v[1];
            _v[2]=v[2];
            _v[3]=v[3];
        }

        inline Quat( const Vec4d& v )
        {
            _v[0]=v[0];
            _v[1]=v[1];
            _v[2]=v[2];
            _v[3]=v[3];
        }

        inline Quat( value_type angle, const Vec3f& axis)
        {
            makeRotate(angle,axis);
        }
        inline Quat( value_type angle, const Vec3d& axis)
        {
            makeRotate(angle,axis);
        }

        inline Quat( value_type angle1, const Vec3f& axis1, 
                     value_type angle2, const Vec3f& axis2,
                     value_type angle3, const Vec3f& axis3)
        {
            makeRotate(angle1,axis1,angle2,axis2,angle3,axis3);
        }

        inline Quat( value_type angle1, const Vec3d& axis1, 
                     value_type angle2, const Vec3d& axis2,
                     value_type angle3, const Vec3d& axis3)
        {
            makeRotate(angle1,axis1,angle2,axis2,angle3,axis3);
        }

        inline Quat& operator = (const Quat& v) { _v[0]=v._v[0];  _v[1]=v._v[1]; _v[2]=v._v[2]; _v[3]=v._v[3]; return *this; }

        inline bool operator == (const Quat& v) const { return _v[0]==v._v[0] && _v[1]==v._v[1] && _v[2]==v._v[2] && _v[3]==v._v[3]; }

        inline bool operator != (const Quat& v) const { return _v[0]!=v._v[0] || _v[1]!=v._v[1] || _v[2]!=v._v[2] || _v[3]!=v._v[3]; }

        inline bool operator <  (const Quat& v) const
        {
            if (_v[0]<v._v[0]) return true;
            else if (_v[0]>v._v[0]) return false;
            else if (_v[1]<v._v[1]) return true;
            else if (_v[1]>v._v[1]) return false;
            else if (_v[2]<v._v[2]) return true;
            else if (_v[2]>v._v[2]) return false;
            else return (_v[3]<v._v[3]);
        }

        /* ----------------------------------
           Methods to access data members
        ---------------------------------- */

        inline Vec4d asVec4() const
        {
            return Vec4d(_v[0], _v[1], _v[2], _v[3]);
        }

        inline Vec3d asVec3() const
        {
            return Vec3d(_v[0], _v[1], _v[2]);
        }

        inline void set(value_type x, value_type y, value_type z, value_type w)
        {
            _v[0]=x;
            _v[1]=y;
            _v[2]=z;
            _v[3]=w;
        }
        
        inline void set(const Vec4f& v)
        {
            _v[0]=v[0];
            _v[1]=v[1];
            _v[2]=v[2];
            _v[3]=v[3];
        }

        inline void set(const Vec4d& v)
        {
            _v[0]=v[0];
            _v[1]=v[1];
            _v[2]=v[2];
            _v[3]=v[3];
        }
        
  /*void set(const Matrixf& matrix);
        
        void set(const Matrixd& matrix);
        
        void get(Matrixf& matrix) const;

        void get(Matrixd& matrix) const;
  */     

        inline value_type & operator [] (int i) { return _v[i]; }
        inline value_type   operator [] (int i) const { return _v[i]; }

        inline value_type & x() { return _v[0]; }
        inline value_type & y() { return _v[1]; }
        inline value_type & z() { return _v[2]; }
        inline value_type & w() { return _v[3]; }

        inline value_type x() const { return _v[0]; }
        inline value_type y() const { return _v[1]; }
        inline value_type z() const { return _v[2]; }
        inline value_type w() const { return _v[3]; }
/** Length squared of the vector = vec . vec */
inline value_type length2(cv::Vec3f _v) const
        {
            return _v[0]*_v[0] + _v[1]*_v[1] + _v[2]*_v[2];
        }

        /** return true if the Quat represents a zero rotation, and therefore can be ignored in computations.*/
        bool zeroRotation() const { return _v[0]==0.0 && _v[1]==0.0 && _v[2]==0.0 && _v[3]==1.0; } 


         /* ------------------------------------------------------------- 
                   BASIC ARITHMETIC METHODS            
        Implemented in terms of Vec4s.  Some Vec4 operators, e.g.
        operator* are not appropriate for quaternions (as
        mathematical objects) so they are implemented differently.
        Also define methods for conjugate and the multiplicative inverse.            
        ------------------------------------------------------------- */
        /// Multiply by scalar 
        inline const Quat operator * (value_type rhs) const
        {
            return Quat(_v[0]*rhs, _v[1]*rhs, _v[2]*rhs, _v[3]*rhs);
        }

        /// Unary multiply by scalar 
        inline Quat& operator *= (value_type rhs)
        {
            _v[0]*=rhs;
            _v[1]*=rhs;
            _v[2]*=rhs;
            _v[3]*=rhs;
            return *this;        // enable nesting
        }

        /// Binary multiply 
        inline const Quat operator*(const Quat& rhs) const
        {
            return Quat( rhs._v[3]*_v[0] + rhs._v[0]*_v[3] + rhs._v[1]*_v[2] - rhs._v[2]*_v[1],
                 rhs._v[3]*_v[1] - rhs._v[0]*_v[2] + rhs._v[1]*_v[3] + rhs._v[2]*_v[0],
                 rhs._v[3]*_v[2] + rhs._v[0]*_v[1] - rhs._v[1]*_v[0] + rhs._v[2]*_v[3],
                 rhs._v[3]*_v[3] - rhs._v[0]*_v[0] - rhs._v[1]*_v[1] - rhs._v[2]*_v[2] );
        }

        /// Unary multiply 
        inline Quat& operator*=(const Quat& rhs)
        {
            value_type x = rhs._v[3]*_v[0] + rhs._v[0]*_v[3] + rhs._v[1]*_v[2] - rhs._v[2]*_v[1];
            value_type y = rhs._v[3]*_v[1] - rhs._v[0]*_v[2] + rhs._v[1]*_v[3] + rhs._v[2]*_v[0];
            value_type z = rhs._v[3]*_v[2] + rhs._v[0]*_v[1] - rhs._v[1]*_v[0] + rhs._v[2]*_v[3];
            _v[3]   = rhs._v[3]*_v[3] - rhs._v[0]*_v[0] - rhs._v[1]*_v[1] - rhs._v[2]*_v[2];

            _v[2] = z;
            _v[1] = y;
            _v[0] = x;

            return (*this);            // enable nesting
        }

        /// Divide by scalar 
        inline Quat operator / (value_type rhs) const
        {
            value_type div = 1.0/rhs;
            return Quat(_v[0]*div, _v[1]*div, _v[2]*div, _v[3]*div);
        }

        /// Unary divide by scalar 
        inline Quat& operator /= (value_type rhs)
        {
            value_type div = 1.0/rhs;
            _v[0]*=div;
            _v[1]*=div;
            _v[2]*=div;
            _v[3]*=div;
            return *this;
        }

        /// Binary divide 
        inline const Quat operator/(const Quat& denom) const
        {
            return ( (*this) * denom.inverse() );
        }

        /// Unary divide 
        inline Quat& operator/=(const Quat& denom)
        {
            (*this) = (*this) * denom.inverse();
            return (*this);            // enable nesting
        }

        /// Binary addition 
        inline const Quat operator + (const Quat& rhs) const
        {
            return Quat(_v[0]+rhs._v[0], _v[1]+rhs._v[1],
                _v[2]+rhs._v[2], _v[3]+rhs._v[3]);
        }

        /// Unary addition
        inline Quat& operator += (const Quat& rhs)
        {
            _v[0] += rhs._v[0];
            _v[1] += rhs._v[1];
            _v[2] += rhs._v[2];
            _v[3] += rhs._v[3];
            return *this;            // enable nesting
        }

        /// Binary subtraction 
        inline const Quat operator - (const Quat& rhs) const
        {
            return Quat(_v[0]-rhs._v[0], _v[1]-rhs._v[1],
                _v[2]-rhs._v[2], _v[3]-rhs._v[3] );
        }

        /// Unary subtraction 
        inline Quat& operator -= (const Quat& rhs)
        {
            _v[0]-=rhs._v[0];
            _v[1]-=rhs._v[1];
            _v[2]-=rhs._v[2];
            _v[3]-=rhs._v[3];
            return *this;            // enable nesting
        }

        /** Negation operator - returns the negative of the quaternion.
        Basically just calls operator - () on the Vec4 */
        inline const Quat operator - () const
        {
            return Quat (-_v[0], -_v[1], -_v[2], -_v[3]);
        }

        /// Length of the quaternion = sqrt( vec . vec )
        value_type length() const
        {
            return sqrt( _v[0]*_v[0] + _v[1]*_v[1] + _v[2]*_v[2] + _v[3]*_v[3]);
        }

        /// Length of the quaternion = vec . vec
        value_type length2() const
        {
            return _v[0]*_v[0] + _v[1]*_v[1] + _v[2]*_v[2] + _v[3]*_v[3];
        }

        /// Conjugate 
        inline Quat conj () const
        { 
             return Quat( -_v[0], -_v[1], -_v[2], _v[3] );
        }

        /// Multiplicative inverse method: q^(-1) = q^*/(q.q^*)
        inline const Quat inverse () const
        {
             return conj() / length2();
         }

      /* -------------------------------------------------------- 
               METHODS RELATED TO ROTATIONS
        Set a quaternion which will perform a rotation of an
        angle around the axis given by the vector (x,y,z).
        Should be written to also accept an angle and a Vec3?

        Define Spherical Linear interpolation method also

        Not inlined - see the Quat.cpp file for implementation
        -------------------------------------------------------- */
        void makeRotate( value_type  angle, 
                          value_type  x, value_type  y, value_type  z );
        void makeRotate ( value_type  angle, const Vec3f& vec );
        void makeRotate ( value_type  angle, const Vec3d& vec );

        void makeRotate ( value_type  angle1, const Vec3f& axis1, 
                          value_type  angle2, const Vec3f& axis2,
                          value_type  angle3, const Vec3f& axis3);
        void makeRotate ( value_type  angle1, const Vec3d& axis1, 
                          value_type  angle2, const Vec3d& axis2,
                          value_type  angle3, const Vec3d& axis3);

        /** Make a rotation Quat which will rotate vec1 to vec2.
            Generally take a dot product to get the angle between these
            and then use a cross product to get the rotation axis
            Watch out for the two special cases when the vectors
            are co-incident or opposite in direction.*/
        void makeRotate( const Vec3f& vec1, const Vec3f& vec2 );
        /** Make a rotation Quat which will rotate vec1 to vec2.
            Generally take a dot product to get the angle between these
            and then use a cross product to get the rotation axis
            Watch out for the two special cases of when the vectors
            are co-incident or opposite in direction.*/
        void makeRotate( const Vec3d& vec1, const Vec3d& vec2 );
    
        void makeRotate_original( const Vec3d& vec1, const Vec3d& vec2 );

        /** Return the angle and vector components represented by the quaternion.*/
        void getRotate ( value_type & angle, value_type & x, value_type & y, value_type & z ) const;

        /** Return the angle and vector represented by the quaternion.*/
        void getRotate ( value_type & angle, Vec3f& vec ) const;

        /** Return the angle and vector represented by the quaternion.*/
        void getRotate ( value_type & angle, Vec3d& vec ) const;

        /** Spherical Linear Interpolation.
        As t goes from 0 to 1, the Quat object goes from "from" to "to". */
        void slerp   ( value_type  t, const Quat& from, const Quat& to);
  
  //Rotate a vector by this quaternion.
        Vec3f operator* (const Vec3f& v) const
        {
            // nVidia SDK implementation
            Vec3f uv, uuv; 
            Vec3f qvec(_v[0], _v[1], _v[2]);
            uv = qvec .cross (v);
            uuv = qvec.cross(uv); 
            uv = Vec3f(uv[0]* ( 2.0f * _v[3] ),uv[1]* ( 2.0f * _v[3] ),uv[2]* ( 2.0f * _v[3] ) );
            uuv = Vec3f(uuv[0]*2.0f , uuv[1]*2.0f,uuv[2]*2.0f);
            return v + uv + uuv; 
        }
               
  //Rotate a vector by this quaternion.
        Vec3d operator* (const Vec3d& v) const
        {
            // nVidia SDK implementation
            Vec3d uv, uuv; 
            Vec3d qvec(_v[0], _v[1], _v[2]);
            uv = qvec .cross( v);
            uuv = qvec .cross( uv); 
	    uv = Vec3f(uv[0]* ( 2.0f * _v[3] ),uv[1]* ( 2.0f * _v[3] ),uv[2]* ( 2.0f * _v[3] ) );
            uuv = Vec3f(uuv[0]*2.0f , uuv[1]*2.0f,uuv[2]*2.0f);

	    //            uv *= ( 2.0f * _v[3] ); 
	    // uuv *= 2.0f; 
            return v + uv + uuv;
        }
        
    protected:
    
};    // end of class prototype



#endif 
