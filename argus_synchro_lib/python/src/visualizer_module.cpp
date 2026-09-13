#include "visualizer_module.h"
#include "ui_interface/visualizer.h"
#include "logger/py_logger.h"

#include <opencv2/core.hpp>
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>

namespace py = pybind11;

// rgb画像のvectorをcv::Matのvectorにキャストする関数
std::vector<cv::Mat> py_rgbimages_2_cvmat(const std::vector<py::array_t<uint8_t>>& rgb_images)
{
    std::vector<cv::Mat> cv2_rgb_images;
    cv2_rgb_images.reserve(rgb_images.size());
    for (size_t i = 0; i < rgb_images.size(); i++)
    {
        // バッファ情報の取得
        py::buffer_info buf = rgb_images.at(i).request();

        // ここでは例としてRGB画像として扱う
        if (buf.ndim != 3)
        {
            throw std::runtime_error("入力のnumpy配列は3次元（高さ, 幅, チャネル）である必要があります");
        }
        int height = static_cast<int>(buf.shape[0]);
        int width = static_cast<int>(buf.shape[1]);
        int channels = static_cast<int>(buf.shape[2]);

        // チャネル数は1もしくは3を想定（ここでは3を想定）
        if (channels != 3 && channels != 1)
        {
            throw std::runtime_error("サポートされるチャネル数は1または3です");
        }

        // numpyは通常row-majorで連続しているので、OpenCVのcv::Matでそのままラップ可能
        int type = (channels == 1) ? CV_8UC1 : CV_8UC3;

        cv::Mat mat(height, width, type, buf.ptr);

        cv2_rgb_images.push_back(mat.clone());
    }

    return cv2_rgb_images;
};

void bind_visualizer(py::module& m)
{
    py::module m_visualizer = m.def_submodule("visualizer");

    py::class_<GodotUIVisualizer>(m_visualizer, "GodotUIVisualizer")
        .def(py::init<const UIIFConf&, int, double, int, bool, double, const std::string&, LoggerFunc>(),
             py::arg("ui_if_config"), py::arg("s_frame"), py::arg("rotation_radius"), py::arg("camera_num"),
             py::arg("has_external_guard"), py::arg("external_guard_offset"), py::arg("status_mmap_path"),
             py::arg("logfunc"))
        .def("update", &GodotUIVisualizer::update, py::arg("ui_if_config"), py::arg("general_config"),
             py::arg("logfunc"))
        .def("summary",
             [](GodotUIVisualizer& visualizer, int isslow, const Eigen::Ref<const Eigen::MatrixXd>& boxes,
                const Eigen::Ref<const Eigen::MatrixXf>& minmax, const Eigen::Ref<const Eigen::VectorXi>& valid_detects,
                     OctoTree octotree_obj, double angle_deg, const std::vector<py::array_t<uint8_t>>& frames,
                const std::vector<CameraDetectionData>& bb_box_data, const std::vector<Camera>& camera,
                Ccol_res collision_clusters, const std::map<int, NodeEntity>& cluster2entity, int ref_t,
                int max_tree_depth, int dialate_point_size, bool octotree_func_on, bool collisiondetection_func_on,
                bool display_octree, bool damp_out, int process_time_ms)
             {
                 std::vector<cv::Mat> cvframes = py_rgbimages_2_cvmat(frames);
                 visualizer.summary(isslow, boxes, minmax, valid_detects, octotree_obj, angle_deg, cvframes,
                                    bb_box_data, camera, collision_clusters, cluster2entity, ref_t, max_tree_depth,
                                    dialate_point_size, octotree_func_on, collisiondetection_func_on, display_octree,
                                    damp_out, process_time_ms);
             })
        .def("close", &GodotUIVisualizer::close_mmap);
}
