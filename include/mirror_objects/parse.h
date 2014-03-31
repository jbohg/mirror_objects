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

#ifndef PARSE_H
#define PARSE_H
#include "vox.h"
#include <opencv2/opencv.hpp>
#ifndef STANDALONE
#include <sensor_msgs/PointCloud2.h>
#endif

float getRGB( float r, float g, float b);
float getPlausFromRGB( float rgb);
int parse_argument (int argc, char** argv, const char* str, bool &exists);
std::vector<int> parse_file_extension_argument (int argc, 
						char** argv, 
						const std::string &extension);
void parsePoints(const char *filename,std::vector<cv::Point3f> &points);
void parsePoints(const char *filename,std::vector<Vox> &points);
#ifndef STANDALONE
void parsePoints(const char *filename,sensor_msgs::PointCloud2 &cloud);
#endif

#endif
