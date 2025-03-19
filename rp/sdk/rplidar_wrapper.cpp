#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "include/rplidar.h"

namespace py = pybind11;
using namespace rp::standalone::rplidar;

class PyRPlidar {
public:
    PyRPlidar() {
        _driver = RPlidarDriver::CreateDriver();
    }

    ~PyRPlidar() {
        disconnect();
        RPlidarDriver::DisposeDriver(_driver);
    }

    bool connect(const std::string& port, int baudrate = 115200) {
        if (IS_FAIL(_driver->connect(port.c_str(), baudrate))) {
            return false;
        }
        return true;
    }

    void disconnect() {
        if (_driver) {
            _driver->stop();
            _driver->disconnect();
        }
    }

    std::map<std::string, std::string> get_info() {
        rplidar_response_device_info_t info;
        std::map<std::string, std::string> result;
        
        if (IS_FAIL(_driver->getDeviceInfo(info))) {
            result["error"] = "Failed to get device info";
            return result;
        }
        
        char serialNumber[50];
        for (int i = 0; i < 16; i++) {
            sprintf(&serialNumber[i*2], "%02X", info.serialnum[i]);
        }
        serialNumber[32] = '\0';
        
        result["model"] = std::to_string((int)info.model);
        result["firmware_version"] = std::to_string(info.firmware_version);
        result["hardware_version"] = std::to_string(info.hardware_version);
        result["serialnum"] = std::string(serialNumber);
        
        return result;
    }

    std::map<std::string, std::string> get_health() {
        rplidar_response_device_health_t health;
        std::map<std::string, std::string> result;
        
        if (IS_FAIL(_driver->getHealth(health))) {
            result["error"] = "Failed to get device health";
            return result;
        }
        
        result["status"] = std::to_string(health.status);
        result["error_code"] = std::to_string(health.error_code);
        
        return result;
    }

    bool start_scan() {
        if (IS_FAIL(_driver->startScan(false, true))) {
            return false;
        }
        return true;
    }

    bool stop() {
        if (IS_FAIL(_driver->stop())) {
            return false;
        }
        return true;
    }

    py::list get_scan_data() {
        rplidar_response_measurement_node_hq_t nodes[8192];
        size_t count = 8192;
        py::list scan_data;
        
        if (IS_FAIL(_driver->grabScanDataHq(nodes, count))) {
            return scan_data;
        }
        
        if (IS_FAIL(_driver->ascendScanData(nodes, count))) {
            return scan_data;
        }
        
        for (size_t i = 0; i < count; i++) {
            float angle = nodes[i].angle_z_q14 * 90.f / 16384.f;
            float distance = nodes[i].dist_mm_q2 / 4.0f;
            float quality = nodes[i].quality;
            bool startBit = nodes[i].flag & RPLIDAR_RESP_MEASUREMENT_SYNCBIT;
            
            py::dict point;
            point["angle"] = angle;
            point["distance"] = distance;
            point["quality"] = quality;
            point["start_bit"] = startBit;
            
            scan_data.append(point);
        }
        
        return scan_data;
    }

private:
    RPlidarDriver* _driver;
};

PYBIND11_MODULE(pyrplidar_c1, m) {
    py::class_<PyRPlidar>(m, "RPlidar")
        .def(py::init<>())
        .def("connect", &PyRPlidar::connect, py::arg("port"), py::arg("baudrate") = 115200)
        .def("disconnect", &PyRPlidar::disconnect)
        .def("get_info", &PyRPlidar::get_info)
        .def("get_health", &PyRPlidar::get_health)
        .def("start_scan", &PyRPlidar::start_scan)
        .def("stop", &PyRPlidar::stop)
        .def("get_scan_data", &PyRPlidar::get_scan_data);
}
