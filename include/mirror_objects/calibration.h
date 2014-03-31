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

#include <mirror_objects/quat.hpp>

class CameraParameters
{
public:

    cv::Point2d GetF() const {   
	return cv::Point2d(mMatrix[0],mMatrix[4]);   }
    cv::Point2d GetC() const {   
	return cv::Point2d(mMatrix[2],mMatrix[5]);   }
    
    double mMatrix[9];
    double mDistortion[4];
//    cv::Mat mMatrix;
//    cv::Mat mDistortion;
};

class StereoParameters
{
public:
    StereoParameters()
    {
	mFundamental = cv::Mat(3,3,CV_64F,cFundamental);
	mLeftCoeffs = cv::Mat(3,3,CV_64F,cLeftCoeffs);
	mRightCoeffs = cv::Mat(3,3,CV_64F,cRightCoeffs);
    }

    float GetB() const {
	return std::sqrt(mTrans.x*mTrans.x + 
			 mTrans.y*mTrans.y + 
			 mTrans.z*mTrans.z);
    }

    CameraParameters mLeftInt;
    CameraParameters mRightInt;

    cv::Point3f mTrans;
    Quat mRotation;
    double cFundamental[9];
    cv::Mat mFundamental;

    cv::Size mWarpSize;

    double cLeftCoeffs[9];
    double cRightCoeffs[9];
    cv::Mat mLeftCoeffs;
    cv::Mat mRightCoeffs;

    float offset;
};

template<typename tChar>
inline std::basic_ostream<tChar> &operator<<(std::basic_ostream<tChar> &pStream,const CameraParameters &pParams)
{
    
    for(int i=0;i<9;i++)
        pStream << pParams.mMatrix[i] << " ";
    for(int i=0;i<4;i++)
        pStream << pParams.mDistortion[i] << " ";
    return pStream;
}

template<typename tChar>
inline std::basic_istream<tChar> &operator>>(std::basic_istream<tChar> &pStream, CameraParameters &pParams)
{
    for(int i=0;i<9;i++)
        pStream >> pParams.mMatrix[i];
    for(int i=0;i<4;i++)
        pStream >> pParams.mDistortion[i];
    return pStream;
    
}

template<typename tChar>
inline std::basic_ostream<tChar> &operator<<(std::basic_ostream<tChar> &pStream,const StereoParameters &pParams)
{
    pStream<< pParams.mLeftInt << "\n";
    pStream<< pParams.mRightInt << "\n";
    pStream<< pParams.mTrans.x << " " 
	   << pParams.mTrans.y << " " 
	   << pParams.mTrans.z << " ";
    pStream << pParams.mRotation.w() << " " 
	    << pParams.mRotation.x() << " " 
	    << pParams.mRotation.y() << " " 
	    << pParams.mRotation.z() << " ";
    pStream<< pParams.mFundamental <<"\n";
   
    pStream<< pParams.mWarpSize.height << " "
	   << pParams.mWarpSize.width << " ";
    
    pStream<< pParams.mLeftCoeffs << " ";
    pStream<< pParams.mRightCoeffs << " ";
    
    return pStream;
}

template<typename tChar>
inline std::basic_istream<tChar> &operator>>(std::basic_istream<tChar> &pStream,StereoParameters &pParams)
{
    pStream >> pParams.mLeftInt;
    pStream >> pParams.mRightInt;
    pStream >> pParams.mTrans.x 
	    >> pParams.mTrans.y 
	    >> pParams.mTrans.z;

    double lQuat[4];
    for(int i=0;i<4;i++)
        pStream >> lQuat[i];
    pParams.mRotation.set(lQuat[1],lQuat[2],lQuat[3],lQuat[0]);

    for(int i=0;i<9;i++)
    {
        pStream >> pParams.cFundamental[i];
    }
    
    pStream >> pParams.mWarpSize.height >> pParams.mWarpSize.width;

    for(int i=0;i<9;i++)
    {
        pStream >> pParams.cLeftCoeffs[i];
    }
        for(int i=0;i<9;i++)
    {
        pStream >> pParams.cRightCoeffs[i];
    }

    return pStream;
}
