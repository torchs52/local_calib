#include "dataclass_module.h"
#include "dataclass/camera.h"

#include <Eigen/Core>
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

void bind_dataclass(py::module& m)
{
    py::module m_dataclass = m.def_submodule("dataclass");
    m.doc() = "pybind11 bindings for UI_interface and UIIFConf";

    py::class_<Camera>(m_dataclass, "Camera")
        .def(py::init<int, int, int, const Eigen::MatrixXd&, const Eigen::Vector3d&, const Eigen::Matrix3d&,
                      const Eigen::Matrix4d&>(),
             py::arg("cam_index"), py::arg("width"), py::arg("height"), py::arg("rvec"), py::arg("tvec"),
             py::arg("ncm1"), py::arg("extrmat"))
        .def_readwrite("cam_index", &Camera::cam_index)
        .def_readwrite("width", &Camera::width)
        .def_readwrite("height", &Camera::height)
        .def_readwrite("rvec", &Camera::rvec)
        .def_readwrite("tvec", &Camera::tvec)
        .def_readwrite("ncm1", &Camera::ncm1)
        .def_readwrite("extrmat", &Camera::extrmat)
        .def(py::pickle(
            [](const Camera& cam)
            {
                py::dict d;
                d["cam_index"] = cam.cam_index;
                d["width"] = cam.width;
                d["height"] = cam.height;
                d["ncm1"] = py::cast(cam.ncm1);
                d["rvec"] = py::cast(cam.rvec);
                d["tvec"] = py::cast(cam.tvec);
                d["extrmat"] = py::cast(cam.extrmat);
                return d;
            },
            [](py::dict d)
            {
                Camera cam(d["cam_index"].cast<int>(), d["width"].cast<int>(), d["height"].cast<int>(),
                           d["rvec"].cast<Eigen::MatrixXd>(), d["tvec"].cast<Eigen::MatrixXd>(),
                           d["ncm1"].cast<Eigen::MatrixXd>(), d["extrmat"].cast<Eigen::MatrixXd>());
                return cam;
            }));

    // CameraDetectionData のバインディング
    py::class_<CameraDetectionData>(m_dataclass, "CameraDetectionData")
        .def(py::init<const Eigen::MatrixXf&, const Eigen::MatrixXf&,
                      const Eigen::Matrix<int64_t, Eigen::Dynamic, Eigen::Dynamic>&, int>(),
             py::arg("boxes"), py::arg("scores"), py::arg("classes"), py::arg("valid_detects"))
        .def_readwrite("boxes", &CameraDetectionData::boxes)
        .def_readwrite("scores", &CameraDetectionData::scores)
        .def_readwrite("classes", &CameraDetectionData::classes)
        .def_readwrite("valid_detects", &CameraDetectionData::valid_detects)
        .def(py::pickle(
            [](const CameraDetectionData& cam_det_data)
            {
                py::dict d;
                d["boxes"] = py::cast(cam_det_data.boxes);
                d["scores"] = py::cast(cam_det_data.scores);
                d["classes"] = py::cast(cam_det_data.classes);
                d["valid_detects"] = cam_det_data.valid_detects;
                return d;
            },
            [](py::dict d)
            {
                CameraDetectionData cam_det_data(
                    d["boxes"].cast<Eigen::MatrixXf>(), d["scores"].cast<Eigen::MatrixXf>(),
                    d["classes"].cast<Eigen::Matrix<int64_t, Eigen::Dynamic, Eigen::Dynamic>>(),
                    d["valid_detects"].cast<int>());
                return cam_det_data;
            }));
}