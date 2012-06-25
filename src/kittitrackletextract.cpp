#include <boost/program_options.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/point_operators.h>
#include <pcl/common/io.h>
#include <pcl/search/organized.h>
#include <pcl/search/octree.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>
#include <pcl/filters/voxel_grid.h>

#include <iostream>
#include <fstream>

#include "tracklets.h"

using namespace pcl;
using namespace std;

namespace po = boost::program_options;

int main(int argc, char **argv){
	///The file to read from.
	string infile;

	///The file to read tracklets from.
	string trackletfile;

	///The file to output to.
	string outfile;

	///The object type to extract
	string objtype;

	// Declare the supported options.
	po::options_description desc("Program options");
	desc.add_options()
		//Options
		("infile", po::value<string>(&infile)->required(), "the file to read a point cloud from")
		("tracklets", po::value<string>(&trackletfile)->required(), "the file to read kitti tracklet annotations from")
		("outfile", po::value<string>(&outfile)->required(), "the file to write the DoN point cloud & normals to")
		("type", po::value<string>(&objtype), "the name of the object type to extract (default: all)")
		;
	// Parse the command line
	po::variables_map vm;
	po::store(po::parse_command_line(argc, argv, desc), vm);

	// Print help
	if (vm.count("help"))
	{
		cout << desc << "\n";
		return false;
	}

	// Process options.
	po::notify(vm);

	Tracklets *tracklets = new Tracklets();
	if (!tracklets->loadFromFile(trackletfile)){
		cerr << "Could not read tracklets file: " << trackletfile << endl;
	}

	// Load cloud in blob format
	sensor_msgs::PointCloud2 blob;
	pcl::io::loadPCDFile (infile.c_str(), blob);

	pcl::PointCloud<PointXYZI>::Ptr cloud (new pcl::PointCloud<PointXYZI>);
	cout << "Loading point cloud...";
	pcl::fromROSMsg (blob, *cloud);
	cout << "done." << endl;

	//For each tracklet, extract the points
	for(int i = 0; i < tracklets->numberOfTracklets(); i++){
		Tracklets::tTracklet* tracklet = tracklets->getTracklet(i);
		if(objtype.empty() || tracklet->objectType == objtype){
			//
		}

	}

    /*pcl::PCDWriter writer;

    // Save DoN features
    writer.write<PointXYZI> (outfile, *points, false);*/

    delete tracklets;
}
