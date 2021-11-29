//
// Created by yewei on 2/27/20.

// This is an unofficial c++ implementation of Scan Context:
// @ARTICLE{ gkim-2019-ral,
//     author = {G. {Kim} and B. {Park} and A. {Kim}},
//     journal = {IEEE Robotics and Automation Letters},
//     title = {1-Day Learning, 1-Year Localization: Long-Term LiDAR Localization Using Scan Context Image},
//     year = {2019},
//     volume = {4},
//     number = {2},
//     pages = {1948-1955},
//     month = {April}
// }
// For more information please visit: https://github.com/irapkaist/scancontext

#ifndef SRC_SCANCONTEXT_H
#define SRC_SCANCONTEXT_H
//#include "utility.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/range_image/range_image.h>
#include <pcl/common/common.h>
#include <pcl/registration/icp.h>

#include <vector>
#include <cmath>
#include <iostream>
#include <fstream>
#include <string>

#pragma once
#include <Eigen/Core>


typedef pcl::PointXYZI  PointType;

struct PointXYZIRPYT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    float roll;
    float pitch;
    float yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

typedef PointXYZIRPYT  PointTypePose;


struct ScanContextBin
{
    std::string robotname;

    double time;

    PointTypePose pose;

    pcl::PointCloud<PointType>::Ptr cloud;

    Eigen::MatrixXf bin;

    Eigen::VectorXf ringkey;
};

class ScanContext {
public:
    ScanContext(int max_range, int num_rings, int num_sectors);
    ScanContextBin ptcloud2bin(pcl::PointCloud<PointType>::Ptr pt_cloud);

private:
    int _max_range;
    int _num_rings;
    int _num_sectors;

    float _gap;
    float _angle_one_sector;

    //pcl::VoxelGrid<PointType> downSizeFilterInput;

    float xy2Theta(float x, float y);
    Eigen::VectorXf scanContext2RingKey(Eigen::MatrixXf _max_bin);
    Eigen::MatrixXf ptCloud2ScanContext(pcl::PointCloud<PointType>::Ptr pt_cloud);
};

//circShift from https://github.com/irapkaist/SC-LeGO-LOAM
Eigen::MatrixXf circShift( Eigen::MatrixXf &_mat, int _num_shift )
{
    // shift columns to right direction 
    assert(_num_shift >= 0);

    if( _num_shift == 0 )
    {
        Eigen::MatrixXf shifted_mat( _mat );
        return shifted_mat; // Early return 
    }

    Eigen::MatrixXf shifted_mat = Eigen::MatrixXf::Zero( _mat.rows(), _mat.cols() );
    for ( int col_idx = 0; col_idx < _mat.cols(); col_idx++ )
    {
        int new_location = (col_idx + _num_shift) % _mat.cols();
        shifted_mat.col(new_location) = _mat.col(col_idx);
    }

    return shifted_mat;

}

#endif //SRC_SCANCONTEXT_H
