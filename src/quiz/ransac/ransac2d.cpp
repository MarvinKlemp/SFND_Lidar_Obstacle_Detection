/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include <cmath>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float maxDistance)
{
	std::unordered_set<int> best_inlier_indicies;
	for (std::size_t iteration = 0; iteration < maxIterations; iteration++) {
        std::unordered_set<int> inlier_indicies;
        std::size_t first = rand() % (cloud->width + 1);
        std::size_t second;
        while ((second = rand() % (cloud->width + 1)) == first)
            ;

        // Ax + By + C = 0
        // (y1−y2)x+(x2−x1)y+(x1∗y2−x2∗y1)
        auto& p1 = cloud->points[first];
        auto& p2 = cloud->points[second];

        auto A = p1.y - p2.y;
        auto B = p2.x - p1.x;
        auto C = (p1.x*p2.y) - (p2.x*p1.y);

        for (std::size_t idx = 0; idx < cloud->width; idx++) {
            auto &p = cloud->points[idx];
            auto distance =
                    std::abs(A*p.x + B*p.y + C) /
                    (std::sqrt(std::pow(A, 2) + std::pow(B, 2)));

            if (distance < maxDistance) {
                inlier_indicies.insert(idx);
            }
        }

        if (inlier_indicies.size() > best_inlier_indicies.size()) {
            best_inlier_indicies = inlier_indicies;
        }
	}

	return best_inlier_indicies;
}

std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float maxDistance)
{
    std::unordered_set<int> best_inlier_indicies;

    for (std::size_t iteration = 0; iteration < maxIterations; iteration++) {
        std::unordered_set<int> inlier_indicies;
        std::size_t reference = rand() % (cloud->width + 1);
        std::size_t plane_second;
        std::size_t plane_first;

        while ((plane_first = rand() % (cloud->width + 1)) == reference)
            ;
        while ((plane_second = rand() % (cloud->width + 1)) == plane_first)
            ;

        // Ax + By + Cz + D = 0
        // normal vector with: <i,j,k>
        // ix + jy + kz −(ix1 + jy1 + kz1) = 0
        // with x,y,z from point p
        auto& p_reference = cloud->points[reference];
        auto& p_plane_first = cloud->points[plane_first];
        auto& p_plane_second = cloud->points[plane_second];

        // get vectors from reference point to p1 and p2
        auto v3_first = pcl::PointXYZ {p_plane_first.x - p_reference.x, p_plane_first.y - p_reference.y, p_plane_first.z - p_reference.z};
        auto v3_second = pcl::PointXYZ {p_plane_second.x - p_reference.x, p_plane_second.y - p_reference.y, p_plane_second.z - p_reference.z};

        // cross product both vetors to get the normal vector
        auto x1 = v3_first.x;
        auto y1 = v3_first.y;
        auto z1 = v3_first.z;
        auto x2 = v3_second.x;
        auto y2 = v3_second.y;
        auto z2 = v3_second.z;
        auto normal_vector = pcl::PointXYZ {
            y1 * z2 - z1 * y2,
            z1 * x2 - x1 * z2,
            x1 * y2 - y1 * x2
        };

        for (std::size_t idx = 0; idx < cloud->width; idx++) {
            auto &p = cloud->points[idx];

            // use hesse normal form to calculate the distance to the plane of p
            auto d_nominator = std::abs(
                 normal_vector.x * p.x +
                 normal_vector.y * p.y +
                 normal_vector.z * p.z -
                 (normal_vector.x * p_reference.x + normal_vector.y * p_reference.y + normal_vector.z * p_reference.z));
            auto d_denominator = std::sqrt(
                std::pow(normal_vector.x, 2) +
                std::pow(normal_vector.y, 2) +
                std::pow(normal_vector.z, 2));
            auto distance = d_nominator / d_denominator;

            if (distance < maxDistance) {
                inlier_indicies.insert(idx);
            }
        }

        if (inlier_indicies.size() > best_inlier_indicies.size()) {
            best_inlier_indicies = inlier_indicies;
        }
    }

    return best_inlier_indicies;
}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();

	std::unordered_set<int> inliers = RansacPlane(cloud, 100, 0.25);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}

	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
