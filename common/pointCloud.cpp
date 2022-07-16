

#include <pcl/common/projection_matrix.h>
#include <pcl/common/io.h>
#include <pcl/PCLHeader.h>
#include <Eigen/Geometry>
#include <string>



#include "pointCloud.h"


namespace pcl {
    namespace common {
        void pcl_PCLHeader(py::module &m_common) {
            py::class_<pcl::PCLHeader>(m_common, "PointCloudHeader")
                    .def_readwrite("seq", &pcl::PCLHeader::seq)
                    .def_readwrite("stamp", &pcl::PCLHeader::stamp)
                    .def_readwrite("frame_id", &pcl::PCLHeader::frame_id);
        }

        void pcl_PointXYZ(py::module &m_common) {
            py::class_<pcl::PointXYZ>(m_common, "PointXYZ")
                    .def(py::init<>())
                    .def(py::init<pcl::PointXYZ>())
                    .def(py::init<float, float, float>())
                    .def("__repr__", [](pcl::PointXYZ &p) {
                        return "[" + std::to_string(p.x) + ", " + std::to_string(p.y) + ", " + std::to_string(p.z) +
                               "]";
                    })
                    .def_readwrite("x", &pcl::PointXYZ::x)
                    .def_readwrite("y", &pcl::PointXYZ::y)
                    .def_readwrite("z", &pcl::PointXYZ::z);
        }
        void pcl_PointCloud_main(py::module &m_common) {
            m_common.doc() = "PCL Common Point Cloud";
            using CloudTypeXYZ = pcl::PointCloud<pcl::PointXYZ>;
            py::class_<CloudTypeXYZ>(m_common, "PointCloudXYZ")
                    .def(py::init<>())
                    .def(py::init<CloudTypeXYZ>())
                    .def(py::init<CloudTypeXYZ, const std::vector<int>>())
                    .def(py::init<uint32_t, uint32_t>())
                    .def_readwrite("header", &CloudTypeXYZ::header)
                    .def_readwrite("points", &CloudTypeXYZ::points)
                    .def_readwrite("width", &CloudTypeXYZ::width)
                    .def_readwrite("height", &CloudTypeXYZ::height)
                    .def_readwrite("is_dense", &CloudTypeXYZ::is_dense)
                    .def_readwrite("sensor_origin_", &CloudTypeXYZ::sensor_origin_)
                    .def_readwrite("sensor_orientation_", &CloudTypeXYZ::sensor_orientation_)
                    .def("get_sensor_orientation_rot", [](CloudTypeXYZ &p) {
                        return p.sensor_orientation_.toRotationMatrix();
                    })
                    .def("get_sensor_orientation_quat", [](CloudTypeXYZ &p) {
                        Eigen::Quaternionf q = p.sensor_orientation_;
                        Eigen::Vector4f v1(q.x(), q.y(), q.z(), q.w());
                        return v1;
                    })
                    .def("at", [](CloudTypeXYZ &cloud, size_t x, size_t y) {
                        return cloud(x, y);
                    }, py::return_value_policy::reference)
                    .def("at", [](CloudTypeXYZ &cloud, size_t x) {
                        return cloud.points.at(x);
                    }, py::return_value_policy::reference)
                    .def("back", [](CloudTypeXYZ &cloud) {
                        return cloud.points.back();
                    }, py::return_value_policy::reference)
                    .def("front", [](CloudTypeXYZ &cloud) {
                        return cloud.points.front();
                    }, py::return_value_policy::reference)

                    .def("clear", &CloudTypeXYZ::clear)
                    .def("empty", &CloudTypeXYZ::empty)
                    .def("erase", [](CloudTypeXYZ &p, CloudTypeXYZ::iterator position) {
                        p.erase(position);
                    }, py::return_value_policy::copy)//TODO: copy maybe?
                    .def("erase", [](CloudTypeXYZ &p, CloudTypeXYZ::iterator first, CloudTypeXYZ::iterator last) {
                        p.erase(first, last);
                    }, py::return_value_policy::copy)//TODO: copy maybe?
                    .def("getMatrixXfMap", [](const CloudTypeXYZ &p) {
                        return p.getMatrixXfMap();
                    })
                    .def("getMatrixXfMap", [](const CloudTypeXYZ &p, int dim, int stride, int offset) {
                        return p.getMatrixXfMap(dim, stride, offset);
                    })
                    .def("insert", [](CloudTypeXYZ &p, CloudTypeXYZ::iterator position, const pcl::PointXYZ &pt) {
                        return p.insert(position, pt);
                    })
                    .def("insert",
                         [](CloudTypeXYZ &p, CloudTypeXYZ::iterator position, size_t n, const pcl::PointXYZ &pt) {
                             return p.insert(position, n, pt);
                         })
                            //            template <class InputIterator> TODO:: figure out how to do template
                            //            .def("insert", [](CloudTypeXYZ &p, InputIterator first, InputIterator last) {
                            //                return p.insert(position, pt);
                            //            })
                    .def("size", &CloudTypeXYZ::size)
                    .def("bool", &CloudTypeXYZ::isOrganized)
                    .def("__call__", [](const CloudTypeXYZ &p, size_t column, size_t row) {
                        return p(column, row);
                    }, py::return_value_policy::reference)
                    .def("__call__", [](const CloudTypeXYZ &p, size_t n) {
                        return p[n];
                    }, py::return_value_policy::reference)
                    .def("__add__", [](CloudTypeXYZ &p1, CloudTypeXYZ &p2) { return p1 + p2; })
                    .def(py::self += py::self)
                    .def("__iter__", [](const CloudTypeXYZ &s) {
                        return py::make_iterator(s.begin(), s.end());
                    }, py::keep_alive<0, 1>())
                    .def("reserve", &CloudTypeXYZ::reserve)
                    .def("resize", &CloudTypeXYZ::resize)
                    .def("swap", &CloudTypeXYZ::swap)
                    .def("push_back", &CloudTypeXYZ::push_back);
        }


        void pybind_common_pointCloud(py::module& m){
            pcl_PCLHeader(m);
            pcl_PointXYZ(m);
            pcl_PointCloud_main(m);
        };
    }// namespace common
}// namespace pcl




