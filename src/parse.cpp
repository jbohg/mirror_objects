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
#include "parse.h"
#include <stdio.h>
#include <string>


float getRGB( float r, float g, float b){
  union{ int intp; float floatp; } a;
  int res = (int(r*255) << 16) | (int(g*255) << 8) | int(b*255);
  a.intp=res;
  float rgb = *(&a.floatp);
  return rgb;
}

float getPlausFromRGB( float rgb) 
{
    int rgb_int = *reinterpret_cast<int*>(&rgb);
    unsigned char colors = ((rgb_int >> 16) & 0xff);
    return (float)colors/255.0f;
}


int
parse_argument (int argc, char** argv, const char* str, bool &exists)
{
  for (int i = 1; i < argc; ++i)
    {
      // Search for the string
      if (strcmp (argv[i], str) == 0)
	{
	  exists = true;
	  return i;
	}
    }
  exists = false;
  return (-1);
}


////////////////////////////////////////////////////////////////////////////////
/** \brief Parse command line arguments for file names. Returns a vector with 
  * file names indices.
  * \param argc the number of command line arguments
  * \param argv the command line arguments
  * \param extension to search for
  */
std::vector<int>
parse_file_extension_argument (int argc, char** argv, 
			       const std::string &extension)
{
  std::vector<int> indices;
  for (int i = 1; i < argc; ++i)
  {
    std::string fname = std::string (argv[i]);
    std::string ext = extension;
    
    // Needs to be at least 4: .ext
    if (fname.size () <= 4)
      continue;
    
    // For being case insensitive
    std::transform (fname.begin (), fname.end (), fname.begin (), tolower);
    std::transform (ext.begin (), ext.end (), ext.begin (), tolower);
    
    // Check if found
    std::string::size_type it;
    if ((it = fname.find (ext)) != std::string::npos)
    {
      // Additional check: we want to be able to differentiate between .p and .png
      if ((ext.size () - (fname.size () - it)) == 0)
        indices.push_back (i);
    }
  }
  return (indices);
}

void parsePoints(const char *filename,std::vector<cv::Point3f> &points){
  FILE *fp=fopen(filename,"r");
  int num_pts=0;
  if(!fp){
    fprintf(stderr,"Can't open %s\n",filename);
    return;
  }
  int ret;
  ret=fscanf(fp,"%d\n",&num_pts);
  int orig_pts_start=points.size();
  
  points.resize (points.size()+num_pts);
  
  for(int i=0; i < num_pts; i++){
    cv::Point3f pt;
    int color_i[3];
    ret=fscanf(fp,"%f %f %f %*d %*d %d %d %d\n",&pt.x,&pt.y,&pt.z,&color_i[0],&color_i[1],&color_i[2]);
    //float rgb = getRGB((color_i[0]/255.0),(color_i[1]/255.0),(color_i[2]/255.0));
    //printf("%d %d %d %f\n",color_i[0],(color_i[1]),(color_i[2]),rgb);
    points[orig_pts_start+i]=pt;
    //cloud.channels[color_chan].values[orig_pts_start+i]=rgb;

  }
  fclose(fp);
}

void parsePoints(const char *filename,std::vector<Vox> &points){
  FILE *fp=fopen(filename,"r");
  int num_pts=0;
  if(!fp){
    fprintf(stderr,"Can't open %s\n",filename);
    return;
  }
  int ret;
  ret=fscanf(fp,"%d\n",&num_pts);
  int orig_pts_start=points.size();
  
  points.resize (points.size()+num_pts);
  
  for(int i=0; i < num_pts; i++){
    cv::Point3f pt;
    cv::Point img_pt;
    int color_i[3];
    ret=fscanf(fp,"%f %f %f %d %d %d %d %d\n",
	       &pt.x, &pt.y, &pt.z, 
	       &img_pt.x, &img_pt.y, 
	       &color_i[0], &color_i[1], &color_i[2]);
    Vox lVox;
    lVox.pos = pt;
    lVox.imgCoord = img_pt;
    for(int c=0;c<3;c++){
      lVox.color[c] = color_i[c];
    }
    lVox.id = 1;
    points[orig_pts_start+i]=lVox;
    
  }
  fclose(fp);
}

#ifndef STANDALONE
void parsePoints(const char *filename, sensor_msgs::PointCloud2 &cloud){
  FILE *fp=fopen(filename,"r");
  int num_pts=0;
  if(!fp){
    fprintf(stderr,"Can't open %s\n",filename);
    return;
  }
  int ret;
  ret=fscanf(fp,"%d\n",&num_pts);
  
  std::cout << "Reading file with " << num_pts << " data points." 
	    << std::endl;

  cloud.header.frame_id = "/narrow_stereo_optical_frame";
  cloud.header.stamp = ros::Time::now();
  cloud.width        = num_pts;
  cloud.height       = 1;
  cloud.is_dense     = true;
  cloud.is_bigendian = false;
  // size of field is 3 for x, y, z and 1 for rgb
  cloud.fields.resize( 3 + 1);
  cloud.fields[0].name = "x"; 
  cloud.fields[1].name = "y"; 
  cloud.fields[2].name = "z";
  cloud.fields[3].name = "rgb"; 
  int offset = 0;
  for (size_t d = 0; d < cloud.fields.size (); ++d, offset += 4) {
    cloud.fields[d].offset = offset;
    cloud.fields[d].datatype = sensor_msgs::PointField::FLOAT32;
    cloud.fields[d].count  = 1;
  }
  
  cloud.point_step = offset;
  cloud.row_step   = cloud.point_step * cloud.width;

  cloud.data.resize (cloud.width * cloud.height * cloud.point_step);
    

  printf("Point Cloud Specs\n point step: %i\nrow_step %i\ndata size %ld",
	   cloud.point_step, cloud.row_step, (long int)cloud.data.size());

  for(unsigned int i=0;i<cloud.fields.size(); i++)
    printf("Offset %i: %i", i, cloud.fields[i].offset);

  for(int i=0; i < num_pts; i++){
    
    float x, y, z;
    int color_i[3];
    ret=fscanf(fp,"%f %f %f %*d %*d %d %d %d\n",
	       &x,&y,&z,&color_i[0],&color_i[1],&color_i[2]);
    float rgb = getRGB((color_i[0]/255.0),
		       (color_i[1]/255.0),
		       (color_i[2]/255.0));
    
    memcpy (&cloud.data[i * cloud.point_step + cloud.fields[0].offset], 
	    &x, sizeof (float));
    memcpy (&cloud.data[i * cloud.point_step + cloud.fields[1].offset], 
	    &y, sizeof (float));
    memcpy (&cloud.data[i * cloud.point_step + cloud.fields[2].offset], 
	    &z, sizeof (float));
    memcpy (&cloud.data[i * cloud.point_step + cloud.fields[3].offset], 
	    &rgb, sizeof (float));

  }
  
  fclose(fp);
}
#endif
