//
// Created by haochen on 7/14/22.
//

#ifndef PYBINDING_POINTCLOUD_IO_H
#define PYBINDING_POINTCLOUD_IO_H
#include "../pybind_pcl.h"
namespace pcl{
    namespace io{
        void pybind_io_pointCloud(pybind11::module& m);
    }// common
}// pcl
#endif //PYBINDING_POINTCLOUD_IO_H
