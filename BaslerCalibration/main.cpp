#include <filesystem>
#include "capture_images.cpp"
#include "calibrate_camera.cpp"

using namespace std;

int main(int /*argc*/, char * /*argv*/[]) {
    cout << endl << R"(Do you want to capture new calibration images? (y/n) )";
    cout.flush();
    string userInput;
    getline(cin, userInput);
    bool capture = false;

    for (char key: userInput) {
        if ((key == 'y' || key == 'Y')) {
            capture = true;
            break;
        } else if ((key == 'n') || (key == 'N')) {
            capture = false;
            break;
        }
    }

    if (capture) {
        // Clear images directory
        filesystem::remove_all(BASLER_DIR "/images/");
        filesystem::create_directory(BASLER_DIR "/images/");
        // Capture Images
        auto exit_code = capture_images();
        if (exit_code != 0) {
            return exit_code;
        }
    }

    cout << "Calibrating..." << endl;

    auto exit_code = calibrate_camera();

    return exit_code;
}