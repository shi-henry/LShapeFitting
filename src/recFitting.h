//
// Created by ubuntu on 6/24/19.
//

#ifndef UNTITLED_RECFITTING_H
#define UNTITLED_RECFITTING_H

#include <pcl/filters/extract_indices.h>
#include <pcl/point_types.h>
#include <vector>
#include <Eigen/Core>

struct lShapeReuslt
{
    /* data */
    Eigen::Vector3f translation;
    Eigen::Quaternionf rotation;
    float with;
    float height;
    float deep;
};



class recFitting {
public:
    void fitting(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudPoints, std::vector<pcl::PointIndices>& cluster_inds);
    std::vector<lShapeReuslt> shapeRlt;

private:
    float calc_closeness_criterion(Eigen::MatrixXf& cluster_2d_vc);
    int calc_boundingBox(float minp[2], Eigen::MatrixXf& cluster_3d);
    void rectangle_search (Eigen::MatrixXf& cluster_2d);


private:
    const float R0 = 3.0F;   // [m] range segmentation param
    const float Rd = 0.001F; // [m] range segmentation param
    const float dtheta_deg_for_serarch = 1.0F; // [deg]
    const float min_dist_of_closeness_crit = 0.01F; // [m]
    std::vector<std::pair<double, double>> edgePoints_2d;

};


#endif //UNTITLED_RECFITTING_H
