//
// Created by haochen on 7/14/22.
//

#ifndef PYBINDING_PCL_H
#define PYBINDING_PCL_H


#include <pybind11/pybind11.h>
#include <pybind11/eigen.h> // Eigen support
#include <pybind11/stl.h> // STL
#include <pybind11/operators.h> // operators

// member files
#include "common/pointCloud.h"
#include "io/pointCloud_io.h"
namespace py = pybind11;
#endif //PYBINDING_PCL_H
