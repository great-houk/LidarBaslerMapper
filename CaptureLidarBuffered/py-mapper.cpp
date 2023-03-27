#include <pybind11/pybind11.h>
#include "LidarCapture.h"

namespace py = pybind11;

PYBIND11_MODULE(pymapper, m) {
    m.def("start", &LidarCapture::init);
    m.def("stop", &LidarCapture::stop);
    m.def("find_sphere", &LidarCapture::findSphere);
    
    m.def("test", [](const std::string &s) { std::cout << s << std::endl; });

    py::class_<sphereCenter>(m, "sphereCenter")
            .def_readonly("x", &sphereCenter::x)
            .def_readonly("y", &sphereCenter::y)
            .def_readonly("z", &sphereCenter::z)
            .def_readonly("r", &sphereCenter::r)
            .def("__repr__", [](const sphereCenter& s){
                return "sphereCenter {\n\tx: " + std::to_string(s.x) + "\n\ty: " + std::to_string(s.y) + "\n\tz: " + std::to_string(s.z) + "\n\tr: " + std::to_string(s.r) + "\n}";
            });

    py::class_<point3d>(m, "point3d")
            .def_readonly("x", &point3d::x)
            .def_readonly("y", &point3d::y)
            .def_readonly("z", &point3d::z)
            .def("__repr__", [](const point3d& s){
                return "point3d {\n\tx: " + std::to_string(s.x) + "\n\ty: " + std::to_string(s.y) + "\n\tz: " + std::to_string(s.z) + "\n}";
            });
}
