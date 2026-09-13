#include "bind_opaque.h"
#include "collision_detector_module.h"
#include "controller_module.h"
#include "edge_det_module.h"
#include "machine_collision_module.h"
#include "octotree_module.h"
#include "dataclass_module.h"
#include "SceneModule.h"
#include "error_mmap_writer_module.h"
#include "status_mmap_module.h"
#include "ui_interface_module.h"
#include "visualizer_module.h"
#include "detect3d_module.h"
#include "spsc_slot_controller_module.h"

#include <optional>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
namespace py = pybind11;

bool is_initialized = false;

PYBIND11_MODULE(argus_synchro_lib, m)
{
    // settings openmp threads
    if (!is_initialized)
    {
        Eigen::initParallel();
        is_initialized = true;
    }
    bind_octotree(m);
    bind_machine_collision(m);
    bind_collision_detector(m);
    bind_controller(m);
    bind_edge_det(m);

    bind_dataclass(m);

    bind_sceneModule(m);

    bind_error_mmap_writer(m);

    bind_status_mmap_interface(m);
    bind_ui_interface(m);
    bind_visualizer(m);

    bind_detect3d(m);
    bind_spsc_slot_controller(m);
}
