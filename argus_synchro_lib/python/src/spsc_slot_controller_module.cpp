#include "spsc_slot_controller_module.h"

#include "controller/spsc_slot_controller.h"

#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>

#include <cstdint>
#include <stdexcept>
#include <tuple>
#include <utility>

namespace py = pybind11;

namespace
{

std::int64_t* validate_state(py::array_t<std::int64_t, py::array::c_style>& state)
{
    py::buffer_info info = state.request();
    if (info.ndim != 1 || info.size != static_cast<py::ssize_t>(SpscSlotController::kStateSize))
    {
        throw std::invalid_argument("state must be int64[4]");
    }
    return static_cast<std::int64_t*>(info.ptr);
}

class PySpscSlotController
{
  public:
    PySpscSlotController(py::array_t<std::int64_t, py::array::c_style> state, int slot_count)
        : state_(std::move(state)), controller_(validate_state(state_), slot_count)
    {
    }

    std::int64_t published_seq() const
    {
        return controller_.published_seq();
    }

    int reserve_write_slot_latest()
    {
        return controller_.reserve_write_slot_latest();
    }

    int reserve_write_slot_sync()
    {
        return controller_.reserve_write_slot_sync();
    }

    void publish(int slot)
    {
        controller_.publish(slot);
    }

    std::tuple<int, std::int64_t> acquire_read_slot_latest(std::int64_t last_seq)
    {
        return controller_.acquire_read_slot_latest(last_seq);
    }

    std::tuple<int, std::int64_t> acquire_read_slot_sync()
    {
        return controller_.acquire_read_slot_sync();
    }

    void release_read_slot(int slot)
    {
        controller_.release_read_slot(slot);
    }

  private:
    py::array_t<std::int64_t, py::array::c_style> state_;
    SpscSlotController controller_;
};

} // namespace

void bind_spsc_slot_controller(py::module& m)
{
    py::module m_spsc_slot_controller = m.def_submodule("spsc_slot_controller");
    m_spsc_slot_controller.doc() = "pybind11 bindings for spsc_slot_controller";

    py::class_<PySpscSlotController>(m_spsc_slot_controller, "SpscSlotController")
        .def(py::init<py::array_t<std::int64_t, py::array::c_style>, int>(), py::arg("state"), py::arg("slot_count"))
        .def("published_seq", &PySpscSlotController::published_seq)
        .def("reserve_write_slot_latest", &PySpscSlotController::reserve_write_slot_latest)
        .def("reserve_write_slot_sync", &PySpscSlotController::reserve_write_slot_sync)
        .def("publish", &PySpscSlotController::publish, py::arg("slot"))
        .def("acquire_read_slot_latest", &PySpscSlotController::acquire_read_slot_latest, py::arg("last_seq"))
        .def("acquire_read_slot_sync", &PySpscSlotController::acquire_read_slot_sync)
        .def("release_read_slot", &PySpscSlotController::release_read_slot, py::arg("slot"));
}
