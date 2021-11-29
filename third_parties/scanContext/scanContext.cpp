//
// Created by yewei on 2/27/20.
//

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

#include "scanContext.h"
ScanContext::ScanContext(int max_range, int num_rings, int num_sectors)
  : _max_range(max_range), _num_rings(num_rings), _num_sectors(num_sectors){
  _gap = float(_max_range) / float(_num_rings);
  _angle_one_sector = 360.0 / float(_num_sectors);
}

ScanContextBin ScanContext::ptcloud2bin(pcl::PointCloud<PointType>::Ptr pt_cloud){
  ScanContextBin sc_bin;
  sc_bin.cloud.reset(new pcl::PointCloud<PointType>());
  std::swap( sc_bin.cloud, pt_cloud);
  //sc_bin.cloud = pt_cloud;
  sc_bin.bin = ptCloud2ScanContext(sc_bin.cloud);
  sc_bin.ringkey = scanContext2RingKey(sc_bin.bin);
  return sc_bin;
}

Eigen::MatrixXf ScanContext::ptCloud2ScanContext(pcl::PointCloud<PointType>::Ptr pt_cloud){
  Eigen::MatrixXf max_bin = Eigen::MatrixXf::Zero(_num_rings, _num_sectors);
  Eigen::MatrixXf bin_counter = Eigen::MatrixXf::Zero(_num_rings, _num_sectors);

  int num_points = pt_cloud->points.size();

  for (int i = 0; i < num_points; ++i){
    //point info
    PointType point_this = pt_cloud->points[i];
    float range = sqrt(point_this.x*point_this.x + point_this.y*point_this.y);
    float theta = xy2Theta(point_this.x, point_this.y);

    //find corresponding ring index
    //ring index: 0 ~ num rings-1
    int ring_index = floor(range/_gap);
    if (ring_index >= _num_rings)
      ring_index = _num_rings - 1;

    //find corresponding sector index
    //sector index: 1 ~ num sectors-1
    int sector_index = ceil(theta/_angle_one_sector);
    if (sector_index == 0)
      continue;
    else if(sector_index > _num_sectors || sector_index < 1)
      sector_index = _num_sectors - 1;
    else
      sector_index = sector_index - 1;

    if (point_this.z > max_bin(ring_index, sector_index))
      max_bin(ring_index, sector_index) = point_this.z;

    bin_counter(ring_index, sector_index)++;
  }

  for (int i = 0; i < _num_rings; i++){
    for (int j = 0; j < _num_sectors; j++){
      if(bin_counter(i,j)<5)
        max_bin(i,j) = 0;
    }
  }
  return max_bin;
}

Eigen::VectorXf ScanContext::scanContext2RingKey(Eigen::MatrixXf sc_bin){
  Eigen::VectorXf ringkey = Eigen::VectorXf::Zero(_num_rings);
  int nonzeros;
  for (int i = 0; i< _num_rings; i++){
    nonzeros = 0;
    for (int j = 0; j < _num_sectors; j++)
      if (sc_bin(i,j) != 0)
        nonzeros++;
    ringkey(i) = float(nonzeros)/float(_num_sectors);
  }
  return ringkey;
}

float ScanContext::xy2Theta(float x, float y){
  if ( x>=0 && y>=0)
    return 180/M_PI * atan(y/x);

  if ( x<0 && y>=0)
    return 180 - ((180/M_PI) * atan(y/(-x)));

  if (x < 0 && y < 0)
    return 180 + ((180/M_PI) * atan(y/x));

  if ( x >= 0 && y < 0)
    return 360 - ((180/M_PI) * atan((-y)/x));
}