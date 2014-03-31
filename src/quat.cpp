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

#include "quat.hpp"
void Quat::makeRotate( const Vec3f& from, const Vec3f& to )
{
    makeRotate( Vec3d(from), Vec3d(to) );
}

/** Make a rotation Quat which will rotate vec1 to vec2

This routine uses only fast geometric transforms, without costly acos/sin computations. 
It's exact, fast, and with less degenerate cases than the acos/sin method.

For an explanation of the math used, you may see for example: 
http://logiciels.cnes.fr/MARMOTTES/marmottes-mathematique.pdf

@note This is the rotation with shortest angle, which is the one equivalent to the 
acos/sin transform method. Other rotations exists, for example to additionally keep 
a local horizontal attitude.

@author Nicolas Brodu
*/

// Set the elements of the Quat to represent a rotation of angle
/// (radians) around the axis (x,y,z)
void Quat::makeRotate( value_type angle, value_type x, value_type y, value_type z )
{
  const value_type epsilon = 0.0000001;
  
  value_type length = sqrt( x*x + y*y + z*z );
  if (length < epsilon)
    {
      // ~zero length axis, so reset rotation to zero.
      *this = Quat();
      return;
    }

  value_type inversenorm  = 1.0/length;
  value_type coshalfangle = cos( 0.5*angle );
  value_type sinhalfangle = sin( 0.5*angle );
  
  _v[0] = x * sinhalfangle * inversenorm;
  _v[1] = y * sinhalfangle * inversenorm;
  _v[2] = z * sinhalfangle * inversenorm;
  _v[3] = coshalfangle;
}


void Quat::makeRotate( value_type angle, const Vec3f& vec )
{
    makeRotate( angle, vec[0], vec[1], vec[2] );
}

void Quat::makeRotate( value_type angle, const Vec3d& vec )
{
    makeRotate( angle, vec[0], vec[1], vec[2] );
}

void Quat::makeRotate( const Vec3d& from, const Vec3d& to )
{

    // This routine takes any vector as argument but normalized 
    // vectors are necessary, if only for computing the dot product.
    // Too bad the API is that generic, it leads to performance loss.
    // Even in the case the 2 vectors are not normalized but same length,
    // the sqrt could be shared, but we have no way to know beforehand
    // at this point, while the caller may know.
    // So, we have to test... in the hope of saving at least a sqrt
    Vec3d sourceVector = from;
    Vec3d targetVector = to;
    
    value_type fromLen2 = length2(from);
    value_type fromLen;
    // normalize only when necessary, epsilon test
    if ((fromLen2 < 1.0-1e-7) || (fromLen2 > 1.0+1e-7)) {
      fromLen = sqrt(fromLen2);
      sourceVector = Vec3d(sourceVector[0]/fromLen,
			   sourceVector[1]/fromLen,
			   sourceVector[2]/fromLen);
    } else fromLen = 1.0;
    
    value_type toLen2 = length2(to);
    // normalize only when necessary, epsilon test
    if ((toLen2 < 1.0-1e-7) || (toLen2 > 1.0+1e-7)) {
        value_type toLen;
        // re-use fromLen for case of mapping 2 vectors of the same length
        if ((toLen2 > fromLen2-1e-7) && (toLen2 < fromLen2+1e-7)) {
            toLen = fromLen;
        } 
        else toLen = sqrt(toLen2);
	targetVector = Vec3d(targetVector[0]/toLen,
			   targetVector[1]/toLen,
			   targetVector[2]/toLen);

    }

    
    // Now let's get into the real stuff
    // Use "dot product plus one" as test as it can be re-used later on
    double dotProdPlus1 = 1.0 + sourceVector.dot( targetVector);
    
    // Check for degenerate case of full u-turn. Use epsilon for detection
    if (dotProdPlus1 < 1e-7) {
    
        // Get an orthogonal vector of the given vector
        // in a plane with maximum vector coordinates.
        // Then use it as quaternion axis with pi angle
        // Trick is to realize one value at least is >0.6 for a normalized vector.
        if (fabs(sourceVector[0]) < 0.6) {
            const double norm = sqrt(1.0 - sourceVector[0] * sourceVector[0]);
            _v[0] = 0.0; 
            _v[1] = sourceVector[2] / norm;
            _v[2] = -sourceVector[1] / norm;
            _v[3] = 0.0;
        } else if (fabs(sourceVector[1]) < 0.6) {
            const double norm = sqrt(1.0 - sourceVector[1] * sourceVector[1]);
            _v[0] = -sourceVector[2] / norm;
            _v[1] = 0.0;
            _v[2] = sourceVector[0] / norm;
            _v[3] = 0.0;
        } else {
            const double norm = sqrt(1.0 - sourceVector[2] * sourceVector[2]);
            _v[0] = sourceVector[1] / norm;
            _v[1] = -sourceVector[0] / norm;
            _v[2] = 0.0;
            _v[3] = 0.0;
        }
    }
    
    else {
        // Find the shortest angle quaternion that transforms normalized vectors
        // into one other. Formula is still valid when vectors are colinear
        const double s = sqrt(0.5 * dotProdPlus1);
         Vec3d tmp = sourceVector.cross( targetVector);
	tmp= Vec3d(tmp[0]/ (2.0*s), tmp[1]/ (2.0*s), tmp[2]/ (2.0*s));
        _v[0] = tmp[0];
        _v[1] = tmp[1];
        _v[2] = tmp[2];
        _v[3] = s;
    }
}



