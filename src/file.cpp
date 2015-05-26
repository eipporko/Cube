#include "file.h"

#include "vao.h"

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
struct vao loadCloud(string pathFile)
{
    typedef pcl::PointCloud<pcl::PointXYZRGBNormal> CloudType;
    CloudType::Ptr cloud (new CloudType);

    
    //Init
    struct vao VAO;
    VAO.numOfTriangles = 0;
    VAO.numOfVertices  = 0;
    
    
    //Open PCD or PLY files
    if (ends_with(pathFile, ".pcd"))
    {
        if (pcl::io::loadPCDFile (pathFile, *cloud) == -1) //* load the file
        {
            PCL_ERROR ("Couldn't read PCD file. \n");
            return VAO;
        }
    }
    else
        if (ends_with(pathFile, ".ply")) {
            
            if (pcl::io::loadPLYFile (pathFile, *cloud) == -1) //* load the file
            {
                PCL_ERROR ("Couldn't read PLY file. \n");
                return VAO;
            }
        }
    
    cout << endl << "Removing NaN Points ..." << endl;
    std::vector<int> indices;
    int pointsBefore = cloud->size();
    pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

    
    if (pointsBefore - cloud->size() > 0)
        cout << "-> Deleted " << (pointsBefore - cloud->size()) << " NaN Points." << endl;

    //VoxelGrid for remove duplicates
    cout << endl << "Downsampling PointCloud with VoxelGrid filter (leafSize = 0.01f) ..." << endl;
    pointsBefore = cloud->size();
    pcl::VoxelGrid<pcl::PointXYZRGBNormal> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (0.01f, 0.01f, 0.01f);
    sor.filter (*cloud);
    cout << "-> Points Before: " << pointsBefore << " , after: " << cloud->size() << endl;
    
    
    if (cloud->is_dense) {
        
        VAO.numOfVertices = cloud->points.size ();
        
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
            
            VAO.vertices.push_back(glm::vec3(cloud->points[i].x/maxDistance,
                                             cloud->points[i].y/maxDistance,
                                             cloud->points[i].z/maxDistance ));
            
            VAO.normals.push_back(glm::normalize(glm::vec3(cloud->points[i].normal_x,
                                                           cloud->points[i].normal_y,
                                                           cloud->points[i].normal_z)));
            
            VAO.colors.push_back(glm::vec3(cloud->points[i].r/255.f,
                                           cloud->points[i].g/255.f,
                                           cloud->points[i].b/255.f));
            
        }

        VAO.mode = GL_POINTS;
    
    }
    else
    {
        PCL_ERROR( "Points are invalid\n" );
    }
        
    return VAO;

}