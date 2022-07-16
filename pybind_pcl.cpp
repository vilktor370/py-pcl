//
// Created by haochen on 7/14/22.
//




#include "pybind_pcl.h"
namespace pcl {
    PYBIND11_MODULE(pcl, m) {
        common::pybind_common_pointCloud(m);
        io::pybind_io_pointCloud(m);
    }

}// namesapce pcl