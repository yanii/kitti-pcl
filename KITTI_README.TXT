###########################################################################
#          THE KITTI VISION BENCHMARK SUITE: RAW DATA RECORDINGS          #
#             Andreas Geiger    Philip Lenz    Raquel Urtasun             #
#                    Karlsruhe Institute of Technology                    #
#                Toyota Technological Institute at Chicago                #
#                             www.cvlibs.net                              #
###########################################################################

This file gives more information about the KITTI raw data recordings.

General information about streams and timestamps
================================================

Each sensor stream is stored in a single folder. The main folder contains
meta information and a timestamp file, listing the timestamp of each frame
of the sequence to nanosecond precision. Numbers in the data stream correspond
to each numbers in each other data stream and to line numbers in the
timestamp file (0-based index), as all data has been synchronized. All
cameras have been triggered directly by the Velodyne laser scanner, while
from the GPS/IMU system (recording at 100 Hz), we have taken the data
information closest to the respective reference frame. For all sequences
'image_00' has been used as the synchronization reference stream.

Rectified color + grayscale stereo sequences
============================================

Our vehicle has been equipped with four cameras: 1 color camera stereo pair
and 1 grayscale camera stereo pair. The color and grayscale cameras are
mounted close to each other (~6 cm), the baseline of both stereo rigs is
approximately 54 cm. We have chosen this setup such that for the left and
right camera we can provide both color and grayscale information. While the
color cameras (obviously) come with color information, the grayscale camera
images have higher contrast and a little bit less noise.

All cameras are synchronized at about 10 Hz with respect to the Velodyne
laser scanner. The trigger is mounted such that camera images coincide
roughly with the Velodyne lasers facing forward (in driving direction).

All camera images are provided as lossless compressed and rectified png
sequences. The native image resolution is 1382x512 pixels and a little bit
less after rectification, for details see the calibration section below.
The opening angle of the cameras (left-right) is approximately 90 degrees.

The camera images are stored in the following directories:

  - 'image_00': left rectified grayscale image sequence
  - 'image_01': right rectified grayscale image sequence
  - 'image_02': left rectified color image sequence
  - 'image_03': right rectified color image sequence

Velodyne 3D laser scan data
===========================

The velodyne point clouds are stored in the folder 'velodyne_points'. To
save space, all scans have been stored as Nx4 float matrix into a binary
file using the following code:

  stream = fopen (dst_file.c_str(),"wb");
  fwrite(data,sizeof(float),4*num,stream);
  fclose(stream);

Here, data contains 4*num values, where the first 3 values correspond to
x,y and z, and the last value is the reflectance information. All scans
are stored row-aligned, meaning that the first 4 values correspond to the
first measurement. Since each scan might potentially have a different
number of points, this must be determined from the file size when reading
the file, where 1e6 is a good enough upper bound on the number of values:

  // allocate 4 MB buffer (only ~130*4*4 KB are needed)
  int32_t num = 1000000;
  float *data = (float*)malloc(num*sizeof(float));

  // pointers
  float *px = data+0;
  float *py = data+1;
  float *pz = data+2;
  float *pr = data+3;

  // load point cloud
  FILE *stream;
  stream = fopen (currFilenameBinary.c_str(),"rb");
  num = fread(data,sizeof(float),num,stream)/4;
  for (int32_t i=0; i<num; i++) {
    point_cloud.points.push_back(tPoint(*px,*py,*pz,*pr));
    px+=4; py+=4; pz+=4; pr+=4;
  }
  fclose(stream);

x,y and y are stored in metric (m) Velodyne coordinates.

GPS/IMU 3D localization unit
============================

The GPS/IMU information is given in a single small text file which is
written for each synchronized frame. Each text file contains 30 values
which are:

  - lat:     latitude of the oxts-unit (deg)
  - lon:     longitude of the oxts-unit (deg)
  - alt:     altitude of the oxts-unit (m)
  - roll:    roll angle (rad),  0 = level, positive = left side up (-pi..pi)
  - pitch:   pitch angle (rad), 0 = level, positive = front down (-pi/2..pi/2)
  - yaw:     heading (rad),     0 = east,  positive = counter clockwise (-pi..pi)
  - vn:      velocity towards north (m/s)
  - ve:      velocity towards east (m/s)
  - vf:      forward velocity, i.e. parallel to earth-surface (m/s)
  - vl:      leftward velocity, i.e. parallel to earth-surface (m/s)
  - vu:      upward velocity, i.e. perpendicular to earth-surface (m/s)
  - ax:      acceleration in x, i.e. in direction of vehicle front (m/s^2)
  - ay:      acceleration in y, i.e. in direction of vehicle left (m/s^2)
  - az:      acceleration in z, i.e. in direction of vehicle top (m/s^2)
  - af:      forward acceleration (m/s^2)
  - al:      leftward acceleration (m/s^2)
  - au:      upward acceleration (m/s^2)
  - wx:      angular rate around x (rad/s)
  - wy:      angular rate around y (rad/s)
  - wz:      angular rate around z (rad/s)
  - wf:      angular rate around forward axis (rad/s)
  - wl:      angular rate around leftward axis (rad/s)
  - wu:      angular rate around upward axis (rad/s)
  - posacc:  velocity accuracy (north/east in m)
  - velacc:  velocity accuracy (north/east in m/s)
  - navstat: navigation status
  - numsats: number of satellites tracked by primary GPS receiver
  - posmode: position mode of primary GPS receiver
  - velmode: velocity mode of primary GPS receiver
  - orimode: orientation mode of primary GPS receiver

Coordinate Systems
==================

The coordinate systems are defined the following way, where directions
are informally given from the drivers view, when looking forward onto
the road:

  - Camera:   x: right,   y: down,  z: forward
  - Velodyne: x: forward, y: left,  z: up
  - GPS/IMU:  x: forward, y: right, z: down

All coordinate systems are right-handed.

Sensor Calibration
==================

The sensor calibration zip archive contains files, storing matrices in
row-aligned order, meaning that the first values correspond to the first
row:

calib_cam_to_cam.txt: Camera-to-camera calibration
--------------------------------------------------

  - S_xx: 1x2 size of image xx before rectification
  - K_xx: 3x3 calibration matrix of camera xx before rectification
  - D_xx: 1x5 distortion vector of camera xx before rectification
  - R_xx: 3x3 rotation matrix of camera xx (extrinsic)
  - T_xx: 3x1 translation vector of camera xx (extrinsic)
  - S_rect_xx: 1x2 size of image xx after rectification
  - R_rect_xx: 3x3 rectifying rotation to make image planes co-planar
  - P_rect_xx: 3x4 projection matrix after rectification

Note: When using this dataset you will most likely need to access only
P_rect_xx, as this matrix is valid for the rectified image sequences.

calib_velo_to_cam.txt: Velodyne-to-camera registration
------------------------------------------------------

  - R: 3x3 rotation matrix
  - T: 3x1 translation vector
  - delta_f: deprecated
  - delta_c: deprecated

R|T takes a point in Velodyne coordinates and transforms it into the
coordinate system of the left video camera. Likewise it serves as a
representation of the Velodyne coordinate frame in camera coordinates.

calib_imu_to_velo.txt: GPS/IMU-to-Velodyne registration
-------------------------------------------------------

  - R: 3x3 rotation matrix
  - T: 3x1 translation vector

R|T takes a point in GPS/IMU coordinates and transforms it into the
coordinate system of the Velodyne scanner. Likewise it serves as a
representation of the GPS/IMU coordinate frame in Velodyne coordinates.

Tracklet Labels
===============

Tracklet labels are stored in XML and can be read / written using the 
C++/MATLAB source code provided with this development kit. For compiling
the code you will need to have a recent version of the boost libraries
installed.

Each tracklet is stored as a 3D bounding box of given height, width and
length, spanning multiple frames. For each frame we have labeled 3D location
and rotation in bird's eye view. Additionally, occlusion / truncation 
information is provided in the form of averaged Mechanical Turk label
outputs. All tracklets are represented in Velodyne coordinates.

Object categories are classified as following:

  - 'Car'
  - 'Van'
  - 'Truck'
  - 'Pedestrian'
  - 'Person (sitting)'
  - 'Cyclist'
  - 'Tram'
  - 'Misc'

Here, 'Misc' denotes all other categories, e.g., 'Trailers' or 'Segways'.

Reading the Tracklet Label XML Files
====================================

This toolkit provides the header 'cpp/tracklets.h', which can be used to
parse a tracklet XML file into the corresponding data structures. Its usage
is quite simple, you can directly include the header file into your code
as follows:

  #include "tracklets.h"
  Tracklets *tracklets = new Tracklets();
  if (!tracklets->loadFromFile(filename.xml))
    <throw an error>
  <do something with the tracklets>
  delete tracklets;

In order to compile this code you will need to have a recent version of the
boost libraries installed and you need to link against
'libboost_serialization'.

'matlab/readTrackletsMex.cpp' is a MATLAB wrapper for 'cpp/tracklets.h'.
It can be build using make.m. Again you need to link against
'libboost_serialization', which might be problematic on newer MATLAB
versions due to MATLAB's internal definitions of libstdc, etc. The latest
version which we know of which works on Linux is 2008b. This is because
MATLAB has changed its pointer representation.

Of course you can also directly parse the XML file using your preferred
XML parser. If you need to create another useful wrapper for the header file
(e.g., for Python) we would be more than happy if you could share it with us).
