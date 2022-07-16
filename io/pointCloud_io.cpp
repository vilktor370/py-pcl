//
// Created by haochen on 7/14/22.
//

//PCL
#include <pcl/io/auto_io.h>


// pybind
#include "pointCloud_io.h"

namespace pcl {
    namespace io {


//        template<typename PointT> TODO: figure out template
        void io_load(pybind11::module &m) {
            // Load a file into pcl::PCLPointCloud2
            m.def("load",
                  [](const std::string &file_name, pcl::PCLPointCloud2 &blob) {
                      return pcl::io::load(file_name, blob);
                  });

            // Load a file into pcl::PointCloud<PointT>

            m.def("load",
                  [](const std::string &file_name, pcl::PointCloud<pcl::PointXYZ> &cloud) {
                      return pcl::io::load(file_name, cloud);
                  });

            // Load a file into pcl::PolygonMesh
            m.def("load",
                  [](const std::string &file_name, pcl::PolygonMesh &mesh) {
                      return pcl::io::load(file_name, mesh);
                  });

            // Load a file into pcl::TextureMesh
            m.def("load",
                  [](const std::string &file_name, pcl::TextureMesh &mesh) {
                      return pcl::io::load(file_name, mesh);
                  });
        }

//        template<typename PointT> TODO: figure out template
        void io_save(pybind11::module &m) {
            // Load a file into pcl::PCLPointCloud2
            m.def("save",
                  [](const std::string &file_name, pcl::PCLPointCloud2 &blob, unsigned precision = 5) {
                      return pcl::io::save(file_name, blob, precision);
                  });

            // Load a file into pcl::PointCloud<PointT>
            m.def("save",
                  [](const std::string &file_name, pcl::PointCloud<pcl::PointXYZ> &cloud) {
                      return pcl::io::save(file_name, cloud);
                  });

            // Load a file into pcl::PolygonMesh
            m.def("save",
                  [](const std::string &file_name, pcl::PolygonMesh &mesh, unsigned precision = 5) {
                      return pcl::io::save(file_name, mesh, precision);
                  });

            // Load a file into pcl::TextureMesh
            m.def("save",
                  [](const std::string &file_name, pcl::TextureMesh &mesh, unsigned precision = 5) {
                      return pcl::io::save(file_name, mesh, precision);
                  });
        }


        template<typename PointT>
        void pybind_io_pointCloud(pybind11::module &m) {
            io_load(m);
            io_save(m);

        }

    }// namespace io
}// namespace pcl