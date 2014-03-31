#include <opencv2/opencv.hpp>
class BoundingBox
{
    public:
    


  typedef cv::Vec3f vec_type;
  typedef float value_type;

    
        /** Minimum extent. (Smallest X, Y, and Z values of all coordinates.) */
        vec_type _min;
        /** Maximum extent. (Greatest X, Y, and Z values of all coordinates.) */
        vec_type _max;

        /** Creates an uninitialized bounding box. */
        inline BoundingBox() : _min(FLT_MAX,FLT_MAX,FLT_MAX),
                        _max(-FLT_MAX,-FLT_MAX,-FLT_MAX) {}
    
        /** Creates a bounding box initialized to the given extents. */
        inline BoundingBox(value_type xmin, value_type ymin, value_type zmin,
                           value_type xmax, value_type ymax, value_type zmax) :
                           _min(xmin,ymin,zmin),
                           _max(xmax,ymax,zmax) {}

        /** Creates a bounding box initialized to the given extents. */
        inline BoundingBox(const vec_type& min,const vec_type& max) : 
                    _min(min),
                    _max(max) {}

        /** Clear the bounding box. Erases existing minimum and maximum extents. */
        inline void init()
        {
	  _min= cv::Vec3f(FLT_MAX,FLT_MAX,FLT_MAX);
	  _max=cv::Vec3f(-FLT_MAX,-FLT_MAX,-FLT_MAX);
        }
        
        /** Returns true if the bounding box extents are valid, false otherwise. */              
        inline bool valid() const
        {
            return _max[0]>=_min[0] &&  _max[1]>=_min[1] &&  _max[2]>=_min[2];
        }

        /** Sets the bounding box extents. */
        inline void set (value_type xmin, value_type ymin, value_type zmin,
                         value_type xmax, value_type ymax, value_type zmax)
        {
            _min=cv::Vec3f(xmin,ymin,zmin);
            _max=cv::Vec3f(xmax,ymax,zmax);
        }

        /** Sets the bounding box extents. */
        inline void set(const vec_type& min,const vec_type& max)
        {
            _min = min;
            _max = max;
        }


        inline value_type& xMin() { return _min[0]; }
        inline value_type xMin() const { return _min[0]; }
 
        inline value_type& yMin() { return _min[1]; }
        inline value_type yMin() const { return _min[1]; }
 
        inline value_type& zMin() { return _min[2]; }
        inline value_type zMin() const { return _min[2]; }

        inline value_type& xMax() { return _max[0]; }
        inline value_type xMax() const { return _max[0]; }
 
        inline value_type& yMax() { return _max[1]; }
        inline value_type yMax() const { return _max[1]; }
 
        inline value_type& zMax() { return _max[2]; }
        inline value_type zMax() const { return _max[2]; }

        /** Calculates and returns the bounding box center. */
        inline const vec_type center() const
        {
	   cv::Vec3f tmp(_min+_max);
	   return cv::Vec3f(tmp[0]*0.5,tmp[1]*0.5,tmp[2]*0.5);
        }
#if 0
        /** Calculates and returns the bounding box radius. */
        inline value_type radius() const
        {
            return sqrt(radius2());
        }

        /** Calculates and returns the squared length of the bounding box radius.
          * Note, radius2() is faster to calculate than radius(). */
        inline value_type radius2() const
        {
            return 0.25*((_max-_min).length2());
        }
#endif
        /** Returns a specific corner of the bounding box.
          * pos specifies the corner as a number between 0 and 7.
          * Each bit selects an axis, X, Y, or Z from least- to
          * most-significant. Unset bits select the minimum value
          * for that axis, and set bits select the maximum. */
        inline const vec_type corner(unsigned int pos) const
        {
            return vec_type(pos&1?_max[0]:_min[0],pos&2?_max[1]:_min[1],pos&4?_max[2]:_min[2]);
        }

        /** Expands the bounding box to include the given coordinate.
          * If the box is uninitialized, set its min and max extents to v. */
        inline void expandBy(const vec_type& v)
        {
            if(v[0]<_min[0]) _min[0] = v[0];
            if(v[0]>_max[0]) _max[0] = v[0];

            if(v[1]<_min[1]) _min[1] = v[1];
            if(v[1]>_max[1]) _max[1] = v[1];

            if(v[2]<_min[2]) _min[2] = v[2];
            if(v[2]>_max[2]) _max[2] = v[2];
        }

        /** Expands the bounding box to include the given coordinate.
          * If the box is uninitialized, set its min and max extents to
          * Vec3(x,y,z). */
        inline void expandBy(value_type x,value_type y,value_type z)
        {
            if(x<_min[0]) _min[0] = x;
            if(x>_max[0]) _max[0] = x;

            if(y<_min[1]) _min[1] = y;
            if(y>_max[1]) _max[1] = y;

            if(z<_min[2]) _min[2] = z;
            if(z>_max[2]) _max[2] = z;
        }

               

        /** Returns the intersection of this bounding box and the specified bounding box. */
  
        /** Returns true if this bounding box contains the specified coordinate. */
        inline bool contains(const vec_type& v) const
        {
            return valid() && 
                   (v[0]>=_min[0] && v[0]<=_max[0]) &&
                   (v[1]>=_min[1] && v[1]<=_max[1]) &&
                   (v[2]>=_min[2] && v[2]<=_max[2]);
        }
};
