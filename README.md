Completing object shape by mirroring
==============

This C++ project predict the complete shape of an object from the depth measurement obtained from a single view.
It assumes planar symmetry and tries to find the best parameters of the symmetry plane. Poisson surface reconstruction is used to reconstruct a mesh model of the estimated complete shape that may facilitate grasp synthesis.

This code has been developed for the project further described in the following publication

**Mind the Gap - Robotic Grasping under Incomplete Observation.** *Jeannette Bohg, Matthew Johnson-Roberson, Beatriz Leon, Javier Felip, Xavi Gratal, Niklas Bergstrom, Danica Kragic and Antonio Morales. IEEE 2011 International Conference on Robotics and Automation, pp.686 - 693.*

This implementation is in a beta (maybe even gamma) stage.

Requirements
----------
The following libraries are required to compile the code:

* ROS (fuerte)
* OpenCV (image I/O, pre-processing)
* PCL (point cloud I/O)

Compilation
------------
The compilation of this code is tested on Ubuntu with ROS fuerte. However the ROS dependency is absolutely minimal and easy to remove. 
PCL is only used for storing point clouds in the PCD format. The most important dependency right now is OpenCV.

```
rosmake 
```

or

```
make -j
```

Usage
------------
Currently, the code assumes as input point clouds in millimeters and in a coordinate frame aligned with the supporting surface.

Estimate full object shape.
```
./mirrorCloud crdfile -p viewx viewy viewz treeDepth targetDir [optional arguments]
 or ./mirrorCloud crdfile -v viewpointFile     treeDepth targetDir [optional arguments]
 or ./mirrorCloud crdfile -t world2CamFile     treeDepth targetDir [optional arguments]
 optional arguments:
 -f cameraCalibration 
 -m maskFile
 -noRot
```

Reconstruct water-tight object mesh from estimated full shape.
```
./meshCloud pcdfile treeDepth targetDir [optional arguments]
 optional arguments: -withPlaus
```

Estimate full shape and reconstruct in one step.
```
./mirrorAndReconstruct crdfile -p viewx viewy viewz treeDepth targetDir [optional arguments]
 or ./mirrorCloud crdfile -v viewpointFile     treeDepth targetDir [optional arguments]
 or ./mirrorCloud crdfile -t world2CamFile     treeDepth targetDir [optional arguments]
 optional arguments:
 -f cameraCalibration 
 -m maskFile
 -noRot
```

Testing
------------
There is a small dataset for one object in the demo folder of the package. 
![](demo/result.png?raw=true)
To test the package you can either estimate the parameters of the symmetry plane

```
cd bin
./mirrorCloud ../demo/segment_315.crd -v ../demo/viewPoint_315.txt 5 . -f ../demo/calib.txt
```

and afterwards reconstruct a water-tight mesh of the completed object shape

```
cd bin
./meshCloud MirroredFullSearch_segment_315.pcd 5 .
```

or you can run everything in one go

```
cd bin
./mirrorAndReconstruct ../demo/segment_315.crd -v ../demo/viewPoint_315.txt 5 . -f ../demo/calib.txt
```

The reconstructed completed point clouds based on the estimated symmetry plane of this example dataset are shown 
below with the original partial point cloud in blue and the mirrored one in red.
![](demo/topscreen.png?raw=true)
![](demo/sidescreen.png?raw=true)

The complete dataset that was used in the above mentioned publication can be downloaded from  
[here]( http://www-amd.is.tuebingen.mpg.de/~bohg/MirrorDatabase.tgz)