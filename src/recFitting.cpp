//
// Created by ubuntu on 6/24/19.
//

#include "recFitting.h"
#include <Eigen/Core>
#include <cmath>
#include <vector>
#include <pcl/visualization/pcl_painter2D.h>
#include <pcl/visualization/pcl_plotter.h>

using namespace std;

float recFitting::calc_closeness_criterion(Eigen::MatrixXf& cluster_2d_vc)
{
    float c1_max = cluster_2d_vc.row(0).maxCoeff();
    float c2_max = cluster_2d_vc.row(1).maxCoeff();
    float c1_min = cluster_2d_vc.row(0).minCoeff();
    float c2_min = cluster_2d_vc.row(1).minCoeff();

    Eigen::VectorXf arrayOfX = cluster_2d_vc.row(0);
    Eigen::VectorXf arrayOfY = cluster_2d_vc.row(1);

    Eigen::VectorXf a = (c1_max * Eigen::VectorXf::Ones(cluster_2d_vc.cols())) - arrayOfX;
    Eigen::VectorXf b = arrayOfX - Eigen::VectorXf::Ones(cluster_2d_vc.cols()) * c1_min;
    Eigen::VectorXf D1 = a.cwiseMin(b);

    Eigen::VectorXf c = Eigen::VectorXf::Ones(cluster_2d_vc.cols()) * c2_max - arrayOfY;
    Eigen::VectorXf d = arrayOfY - Eigen::VectorXf::Ones(cluster_2d_vc.cols()) * c2_min;
    Eigen::VectorXf D2 = c.cwiseMin(d);

    Eigen::VectorXf minDis = min_dist_of_closeness_crit * Eigen::VectorXf::Ones(cluster_2d_vc.cols());


    Eigen::VectorXf D3 = D1.cwiseMin(D2);
    Eigen::VectorXf D4 = D3.cwiseMax(minDis);
    Eigen::VectorXf D5 = D4.cwiseInverse();
    float distance = D5.sum();
    return distance;

}

int recFitting::calc_boundingBox(float minp[2], Eigen::MatrixXf& cluster_3d)
{
    Eigen::Matrix2f minTrans;
    minTrans << cosf(minp[1]), sinf(minp[1]), -sinf(minp[1]), cosf(minp[1]);

    // x,y,z to x,y
    Eigen::MatrixXf cluster_2d = cluster_3d.topRows(2);
    // trans to object coordinate
    Eigen::MatrixXf cluster_final = minTrans * cluster_2d;

    // get the object border in object coordinate
    float c1_max = cluster_final.row(0).maxCoeff();
    float c2_max = cluster_final.row(1).maxCoeff();
    float c1_min = cluster_final.row(0).minCoeff();
    float c2_min = cluster_final.row(1).minCoeff();
    float c3_max = cluster_3d.row(2).maxCoeff();
    float c3_min = cluster_3d.row(2).minCoeff();

    Eigen::Vector2f centerPoint;
    centerPoint << (c1_max + c1_min)/2.0f, (c2_max + c2_min)/2.0f;

    // trans center point to vehicle coordinate
    centerPoint = minTrans.inverse() * centerPoint;

    lShapeReuslt tmpRlt;
    // calculate translation matrix and box size
    tmpRlt.with = c2_max - c2_min;
    tmpRlt.deep = c1_max - c1_min;
    tmpRlt.height = c3_max - c3_min;
    tmpRlt.translation << centerPoint(0), centerPoint(1), (c3_max + c3_min) / 2.0f;

    // calculate ratation matrix
    // TBD: the sign is opposite to right hand coordinate system
    Eigen::Matrix3f r_matrix = Eigen::Matrix3f::Identity();
    r_matrix << cosf(-minp[1]), sinf(-minp[1]), 0,
            -sinf(-minp[1]), cosf(-minp[1]), 0,
            0, 0, 1;
    tmpRlt.rotation = r_matrix;

    shapeRlt.push_back(tmpRlt);


    // trans border to vehicle coordinate
    Eigen::MatrixXf edgePoint(2, 5); // plot 4 lines need 5 points
    edgePoint << c1_max, c1_max, c1_min, c1_min, c1_max, c2_max, c2_min, c2_min, c2_max, c2_max;
    Eigen::MatrixXf boxEdge = minTrans.inverse() * edgePoint;
    this->edgePoints_2d = {{boxEdge(0,0), boxEdge(1,0)}, {boxEdge(0,1), boxEdge(1,1)},
                                             {boxEdge(0,2), boxEdge(1,2)}, {boxEdge(0,3), boxEdge(1,3)}, {boxEdge(0,4), boxEdge(1,4)}};
}

void recFitting::rectangle_search(Eigen::MatrixXf& cluster_3d)
{
    float dtheta = dtheta_deg_for_serarch * (M_PI / 180.0F);
    float minp[2] = {-FLT_MAX, 0.0};

    // x,y,z to x,y
    Eigen::MatrixXf cluster_2d = cluster_3d.topRows(2);

    for (float theta = 0.0F; theta <= M_PI / 2.0F; theta += dtheta)
    {

        Eigen::Matrix2f transM;
        transM << cosf(theta), sinf(theta), -sinf(theta), cosf(theta);

        Eigen::MatrixXf cluster_vc = transM * cluster_2d; // translate to object coordinate

        float cost = calc_closeness_criterion(cluster_vc);

        if (minp[0] < cost)
        {
            minp[0] = cost;
            minp[1] = theta;
        }
    }

    calc_boundingBox(minp, cluster_3d);

}

void recFitting::fitting(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudPoints, std::vector<pcl::PointIndices>& cluster_inds)
{
    pcl::visualization::PCLPlotter myPclPlotter;
    myPclPlotter.setColorScheme(2);
    // myPclPlotter.setBackgroundColor(0,0,0);
    myPclPlotter.setWindowSize(600, 600);
    myPclPlotter.setYRange(-20,20);
    myPclPlotter.setXRange(-40,0);

    for (const auto& it : cluster_inds)
    {
        Eigen::MatrixXf cluster_3d(3, it.indices.size());
        int index = 0;

        vector<double> plotX;
        vector<double> plotY;

        for (const auto& pit : it.indices)
        {
            cluster_3d(0, index) = cloudPoints->points[pit].x;
            cluster_3d(1, index) = cloudPoints->points[pit].y;
            cluster_3d(2, index) = cloudPoints->points[pit].z;


            // add plot clutered cloud points
            plotX.push_back(cloudPoints->points[pit].x);
            plotY.push_back(cloudPoints->points[pit].y);

            ++index;
        }

        // add plot clutered cloud points
        myPclPlotter.addPlotData(plotX, plotY, "box-2d", vtkChart::POINTS);

        rectangle_search(cluster_3d);

        myPclPlotter.addPlotData(this->edgePoints_2d, "box-2d", vtkChart::LINE,{-1,0,0});
    }
    // plot 2d point cloud and bounding box
    myPclPlotter.plot();
}