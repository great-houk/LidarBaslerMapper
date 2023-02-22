#include "opencv2/opencv.hpp"
#include <pylon/PylonIncludes.h>
#include "../ImagePCDGrabber/ConfigurationEventPrinter.h"
#include "../BaslerCalibration/basler_config.h"

// Namespace for using pylon objects.
using namespace Pylon;

// Namespace for using cout.
using namespace std;

cv::Mat imageMat;
CPylonImage image;
volatile bool LOCK = false;

class ImageGrabHandler : public CImageEventHandler {
public:
    void OnImageGrabbed(CInstantCamera & /*camera*/, const CGrabResultPtr &ptrGrabResult) override {
        while(LOCK) {};
        LOCK = true;
        image.AttachGrabResultBuffer(ptrGrabResult);
        imageMat = cv::Mat((int)image.GetHeight(), (int)image.GetWidth(), CV_8UC3, (uint8_t*)image.GetBuffer());
        LOCK = false;
    }
};

class BaslerCapturer {
public:
    CInstantCamera camera;

    BaslerCapturer() = default;

    void startGrabbing() {
        // Before using any pylon methods, the pylon runtime must be initialized.
        PylonInitialize();

        try {
            // Create an instant camera object for the camera device found first.
            camera.Attach(CTlFactory::GetInstance().CreateFirstDevice());

            // Register the standard configuration event handler for enabling software triggering.
            // The software trigger configuration handler replaces the default configuration
            // as all currently registered configuration handlers are removed by setting the registration mode to RegistrationMode_ReplaceAll.
            camera.RegisterConfiguration(new BaslerLidarConfiguration(false), RegistrationMode_ReplaceAll, Cleanup_Delete);

            // Add image handler
            camera.RegisterImageEventHandler(new ImageGrabHandler(), RegistrationMode_Append, Cleanup_Delete);

            // For demonstration purposes only, add a sample configuration event handler to print out information
            // about camera use.
            camera.RegisterConfiguration(new CConfigurationEventPrinter, RegistrationMode_Append, Cleanup_Delete);

            // Open the camera device.
            camera.Open();

            // Start Grabbing
            camera.StartGrabbing(Pylon::GrabStrategy_LatestImageOnly, Pylon::GrabLoop_ProvidedByInstantCamera);
        }
        catch (const GenericException &e) {
            // Error handling.
            cerr << "An exception occurred." << endl << e.GetDescription() << endl;

            PylonTerminate();
        }
    }

    static cv::Mat getMat() {
        // Wait for the lock to release
        while(LOCK) {}
        LOCK = true;
        auto clone = imageMat.clone();
        LOCK = false;
        return clone;
    }
};