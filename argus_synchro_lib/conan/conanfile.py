"""Conan 2 recipe: argus_synchro_lib dependency graph (includes open3d)."""

from __future__ import annotations

from conan.tools.cmake import CMakeDeps, CMakeToolchain
from conan.tools.gnu import PkgConfigDeps

from conan import ConanFile

_OPEN3D_VERSION = "0.19.0"

_VERSIONS = {
    "nlohmann_json": "3.12.0",
    "nanoflann": "1.9.0",
    "opencv": "4.10.0",
    "pybind11": "3.0.1",
}


class ArgusSynchroLibConan(ConanFile):
    name = "argus-synchro-lib"
    version = "0.1.0"
    description = "argus_synchro_lib and dependencies via Conan 2."
    license = "MIT"
    settings = "os", "arch", "compiler", "build_type"

    def requirements(self) -> None:
        self.requires(f"open3d/{_OPEN3D_VERSION}")
        self.requires(f"nlohmann_json/{_VERSIONS['nlohmann_json']}")
        self.requires(f"nanoflann/{_VERSIONS['nanoflann']}", override=True)
        self.requires("libjpeg-turbo/3.0.4", override=True)
        self.requires(f"opencv/{_VERSIONS['opencv']}")
        self.requires(f"pybind11/{_VERSIONS['pybind11']}")

    def build_requirements(self) -> None:
        self.tool_requires("cmake/[>=3.24 <4]")
        self.tool_requires("ninja/[>=1.10]")

    def configure(self) -> None:
        ocv = self.options["opencv"]
        ocv.contrib = False
        ocv.with_jpeg = "libjpeg-turbo"
        ocv.with_ffmpeg = False
        ocv.with_gtk = False
        ocv.with_wayland = False
        ocv.with_qt = False
        ocv.with_openmp = True
        ocv.with_ipp = False
        ocv.with_itt = False
        ocv.with_eigen = True
        ocv.with_protobuf = False
        ocv.with_opencl = False
        ocv.with_cuda = False
        ocv.with_highgui = False
        ocv.with_video = False
        ocv.with_photo = False
        ocv.with_objdetect = False
        ocv.with_stitching = False
        ocv.with_ml = False
        ocv.with_dnn = False
        ocv.with_features2d = False
        ocv.with_flann = False
        ocv.with_imgproc = True
        ocv.with_calib3d = True
        ocv.with_core = True

    def generate(self) -> None:
        tc = CMakeToolchain(self)
        tc.generate()

        deps = CMakeDeps(self)
        deps.set_property("open3d", "cmake_file_name", "Open3D")
        deps.set_property("open3d", "cmake_target_name", "Open3D::Open3D")
        deps.generate()
        PkgConfigDeps(self).generate()
