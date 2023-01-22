#include "capture_lidar.cpp"
#include "capture_basler.cpp"

int main() {
    auto ret = capture_basler();
    if(ret != 0) {
        return ret;
    }
    ret = capture_lidar();

    return ret;
}