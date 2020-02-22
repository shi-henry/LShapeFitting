/*
 * objectFitting.cpp
 *
 *  Created on: Mar 6, 2019
 *      Author: ubuntu
 */

#include <iostream>
#include <cstdlib>
#include <string>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <chrono>

#include "recFitting.h"


void viewerOneOff(pcl::visualization::PCLVisualizer& viewer)
{
    viewer.setBackgroundColor(0, 0, 1);
    viewer.addCoordinateSystem();
}


void readPonitCloud(const std::string& fileName, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    std::fstream input(fileName.c_str(), std::ios::in | std::ios::binary);
    if(!input.good())
    {
        std::cerr << "could not read file: "<< fileName <<endl;
        exit(0);
    }

    // pcl::PCDReader reader;
    // reader.read("3dpoints.pcd", *cloud);

    int i;
    for(i = 0; input.good() && !input.eof(); ++i)
    {
        pcl::PointXYZ point;
        float I;
        input.read((char*)&point.x, 3*sizeof(float));
        input.read((char*)&I, sizeof(float));
        cloud->push_back(point);
    }
    input.close();

    // pcl::PCDWriter writer;
    // writer.write("hehe.pcd", *cloud, false);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr planeSeg(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{


    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);

    seg.setDistanceThreshold(0.2);

    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    if(inliers->indices.size() == 0)
    {
        PCL_ERROR("could not estimate a planar model for the given dataset.");
        return cloud_filtered;
    }

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setNegative(true);
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.filter(*cloud_filtered);
    for (auto iterCld = cloud_filtered->begin(); iterCld != cloud_filtered->end();)
    {
    	float height = coefficients->values[0]* (*iterCld).x + coefficients->values[1]* (*iterCld).y
    	        + coefficients->values[2]* (*iterCld).z + coefficients->values[3];
    	if (height > 2.0F)
    	{
    		iterCld = cloud_filtered->erase(iterCld);
    	}
    	else
    	{
    		++iterCld;
    	}

    }
     std::cout << "Point cloud data: " << cloud->points.size() << "points" <<std::endl;


    return cloud_filtered;
}

void euclideanCluster(const pcl::PointCloud<pcl::PointXYZ>::Ptr& filteredCld,
        std::vector<pcl::PointIndices>& cluter_indices)
{
    auto now = std::chrono::steady_clock::now();

    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.3);
    ec.setMinClusterSize(40);

    ec.setInputCloud(filteredCld);
    auto now3 = std::chrono::steady_clock::now();
    ec.extract(cluter_indices); // conducting EuclideanCluster

    auto now2 = std::chrono::steady_clock::now();
    using duration_type = std::chrono::duration<double>;
    duration_type time_span = std::chrono::duration_cast<duration_type>(now2 - now3);
    std::cout<< "time elapsed: " <<time_span.count()<<endl;
}

void filterClusterByShape(pcl::PointCloud<pcl::PointXYZ>::Ptr& filteredCld,
        std::vector<pcl::PointIndices>& cluter_indices)
{
    for (auto it = cluter_indices.begin(); it != cluter_indices.end();)
    {
        bool insertFlg = true;
        float maxZ, maxX, maxY;
        float minZ, minX, minY;
        maxZ = maxX = maxY = -1000.0F;
        minZ = minX = minY = 1000.0F;

        for (auto pit = it->indices.begin(); pit != it->indices.end(); ++pit)
        {
            maxX = (filteredCld->points[*pit].x > maxX) ? filteredCld->points[*pit].x : maxX;
            minX = (filteredCld->points[*pit].x < minX) ? filteredCld->points[*pit].x : minX;
            maxY = (filteredCld->points[*pit].y > maxY) ? filteredCld->points[*pit].y : maxY;
            minY = (filteredCld->points[*pit].y < minY) ? filteredCld->points[*pit].y : minY;
            maxZ = (filteredCld->points[*pit].z > maxZ) ? filteredCld->points[*pit].z : maxZ;
            minZ = (filteredCld->points[*pit].z < minZ) ? filteredCld->points[*pit].z : minZ;
        }

        if (((maxZ - minZ) > 3.0F) || ((maxX - minX) + (maxY - minY) > 10.0F) || ((maxZ - minZ) < 1.0F) || (maxZ > 3.0F))
        {
            insertFlg = false;
        }
        else
        {
            insertFlg = true;
        }

        if (insertFlg)
        {
            ++it;
        }
        else
        {
            it = cluter_indices.erase(it);
        }

    }
}

int main(int arvc, char** argv)
{
    if (2 != arvc)
    {
        cout<<"the correct command format is: CMD file."<<endl;
        return 0;
    }

    std::string fileName(argv[1]);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<pcl::PointIndices> cluter_indices;

    readPonitCloud(fileName, cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCld = planeSeg(cloud);

    euclideanCluster(filteredCld, cluter_indices);

    filterClusterByShape(filteredCld, cluter_indices);

    pcl::visualization::PCLVisualizer viewer("Filtered");
    viewer.setBackgroundColor(0,0,1);
    viewer.addCoordinateSystem();
    viewer.addPointCloud(filteredCld);

    recFitting fit;
    fit.fitting(filteredCld, cluter_indices);

    for (int i = 0; i < fit.shapeRlt.size(); ++i)
    {
        viewer.addCube(fit.shapeRlt[i].translation, fit.shapeRlt[i].rotation, fit.shapeRlt[i].deep,
                       fit.shapeRlt[i].with, fit.shapeRlt[i].height, std::to_string(i));
        viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0,0.0,0.0,std::to_string(i));
        viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME,std::to_string(i));
    }
    
    while(!viewer.wasStopped())
    {
        viewer.spinOnce(100);
    }


    return 0;
}
