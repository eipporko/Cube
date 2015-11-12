/*
 *
 * CUBE
 *
 * Copyright (c) David Antunez Gonzalez 2013-2015 <dantunezglez@gmail.com>
 * Copyright (c) Luis Omar Alvarez Mures 2013-2015 <omar.alvarez@udc.es>
 * Copyright (c) Emilio Padron Gonzalez 2013-2015 <emilioj@gmail.com>
 *
 * All rights reserved.
 *
 * This file is part of ToView.
 *
 * CUBE is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this library.
 *
 */

#include "file.h"

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/common/distances.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>

#include <fstream>


bool ends_with(const std::string &filename, const std::string &ext)
{
    return ext.length() <= filename.length() &&
    std::equal(ext.rbegin(), ext.rend(), filename.rbegin());
}


char* loadFile(string fname, GLint &fSize)
{
	ifstream::pos_type size;
	char * memblock;
	string text;

	// file read based on example in cplusplus.com tutorial
	ifstream file (fname, ios::in|ios::binary|ios::ate);
	if (file.is_open())
	{
		size = file.tellg();
		fSize = (GLuint) size;
		memblock = new char [size];
		file.seekg (0, ios::beg);
		file.read (memblock, size);
		file.close();

		text.assign(memblock);
	}
	else
	{
		cout << "Unable to open file " << fname << endl;
		exit(1);
	}
	return memblock;
}


//Return VAO with .numOfTrianges = 0 && numOfVertices = 0 if error
VAO loadCloud(string pathFile)
{
    typedef pcl::PointCloud<pcl::PointXYZRGBNormal> CloudType;
    CloudType::Ptr cloud (new CloudType);

    VAO vao;

    //Open PCD or PLY files
    if (ends_with(pathFile, ".pcd"))
    {
        if (pcl::io::loadPCDFile (pathFile, *cloud) == -1) //* load the file
        {
            PCL_ERROR ("Couldn't read PCD file. \n");
            return vao;
        }
    }
    else
        if (ends_with(pathFile, ".ply")) {

            if (pcl::io::loadPLYFile (pathFile, *cloud) == -1) //* load the file
            {
                PCL_ERROR ("Couldn't read PLY file. \n");
                return vao;
            }
        }

    cout << endl << "Removing NaN Points ..." << endl;
    std::vector<int> indices;
    int pointsBefore = cloud->size();
    pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);


    if (pointsBefore - cloud->size() > 0)
        cout << "-> Deleted " << (pointsBefore - cloud->size()) << " NaN Points." << endl;

    cout << "Points: " << cloud->size() << endl;

    if (cloud->is_dense) {

        //Move to origin
        cout << endl << "Centering Cloud to origin ..." << endl;
        CloudType::Ptr centeredCloud (new CloudType);
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid (*cloud, centroid);
        pcl::demeanPointCloud<pcl::PointXYZRGBNormal> (*cloud, centroid, *cloud);

        //Get MaxDistance for Scaling
        cout << endl << "Scaling ..." << endl;
        Eigen::Vector4f farestPoint;
        pcl::compute3DCentroid(*cloud, centroid);
        pcl::getMaxDistance(*cloud, centroid, farestPoint);
        float maxDistance = max( max( abs(farestPoint.x()), abs(farestPoint.y()) ) , abs(farestPoint.z()) );

        //Compute Normals if it's needed
        cout << endl << "Analizing scene normals ..." << endl;

        bool isNeededNormalsEstimation = false;
        for (size_t i = 0; i < cloud->points.size(); i++) {
            //This normal is not valid.
            if (cloud->points[i].normal_x == 0 && cloud->points[i].normal_y == 0 && cloud->points[i].normal_z == 0) {
                cout << "-> Failed to find valid normals on the pointCloud." << endl;
                isNeededNormalsEstimation = true;
                break;
            }
        }

        if (isNeededNormalsEstimation) {
            cout << "-> Estimating scene normals ..." << endl;
            pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBNormal> ());
            pcl::NormalEstimationOMP<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> NEmop(0);
            NEmop.setInputCloud(cloud->makeShared());
            NEmop.setSearchMethod(tree);
            NEmop.setKSearch(20);
            NEmop.compute(*cloud);
        }



        //Pushing data cloud to VAO structure
        for (size_t i = 0; i < cloud->points.size (); ++i) {
            cloud->points[i].x = cloud->points[i].x/maxDistance;
            cloud->points[i].y = cloud->points[i].y/maxDistance;
            cloud->points[i].z = cloud->points[i].z/maxDistance;
        }

        return VAO(cloud);

    }
    else
    {
        PCL_ERROR( "Points are invalid\n" );
    }

    return vao;

}
