// Include files to use the pylon API.
#include <pylon/PylonIncludes.h>

#include <opencv2/opencv.hpp>

// Include Config Class
#include "basler_config.h"

// Namespace for using pylon objects.
using namespace Pylon;

using namespace std;
using namespace cv;

// Image used between threads
CPylonImage image;

//Example of an image event handler.
class ImageGrabHandler : public CImageEventHandler {
public:
    void OnImageGrabbed(CInstantCamera & /*camera*/, const CGrabResultPtr &ptrGrabResult) override {
        cout << "Grabbed Image" << std::endl;

        // Initializes the image object with the buffer from the grab result.
        // This prevents the reuse of the buffer for grabbing as long as it is
        // not released.
        // Please note that this is not relevant for this example because the
        // camera object has been destroyed already.
        image.AttachGrabResultBuffer(ptrGrabResult);
    }
};

int capture_images() {
    // The exit code of the sample application.
    int exitCode = 0;

    // Before using any pylon methods, the pylon runtime must be initialized.
    PylonInitialize();

    try {
        // Create an instant camera object for the camera device found first.
        CInstantCamera camera(CTlFactory::GetInstance().CreateFirstDevice());

        // Register the standard configuration event handler for enabling software triggering.
        // The software trigger configuration handler replaces the default configuration
        // as all currently registered configuration handlers are removed by setting the registration mode to RegistrationMode_ReplaceAll.
        camera.RegisterConfiguration(new BaslerLidarConfiguration(true), RegistrationMode_ReplaceAll, Cleanup_Delete);

        // For demonstration purposes only, register another image event handler.
        camera.RegisterImageEventHandler(new ImageGrabHandler, RegistrationMode_Append, Cleanup_Delete);

        // Open the camera device.
        camera.Open();

        // Can the camera device be queried whether it is ready to accept the next frame trigger?
        if (camera.CanWaitForFrameTriggerReady()) {
            // Start the grabbing using the grab loop thread, by setting the grabLoopType parameter
            // to GrabLoop_ProvidedByInstantCamera. The grab results are delivered to the image event handlers.
            // The GrabStrategy_OneByOne default grab strategy is used.
            camera.StartGrabbing(GrabStrategy_OneByOne, GrabLoop_ProvidedByInstantCamera);

            // Wait for user input to trigger the camera or exit the program.
            // The grabbing is stopped, the device is closed and destroyed automatically when the camera object goes out of scope.

            bool runLoop = true;
            int count = 0;
            while (runLoop) {
                cout << endl << R"(Enter "t" to trigger the camera or "e" to exit and press enter? (t/e) )";
                cout.flush();

                string userInput;
                getline(cin, userInput);

                for (char key: userInput) {
                    if ((key == 't' || key == 'T')) {
                        // Execute the software trigger. Wait up to 1000 ms for the camera to be ready for trigger.
                        if (camera.WaitForFrameTriggerReady(1000, TimeoutHandling_ThrowException)) {
                            camera.ExecuteSoftwareTrigger();

                            // Wait some time to allow the OnImageGrabbed handler print its output,
                            // so the printed text on the console is in the expected order and
                            // also to allow the image to be updated.
                            WaitObject::Sleep(250);

                            // Save file
                            String_t filename = BASLER_DIR"/images/image_";
                            for (char c: to_string(count) + ".png") {
                                filename.append(1, c);
                            }
                            image.Save(ImageFileFormat_Png, filename);
                            count++;

                            // Display image
                            CImageFormatConverter fc;
                            fc.OutputPixelFormat = PixelType_BGR8packed;

                            CPylonImage convertedImage;
                            fc.Convert(convertedImage, image);
                            Mat cv_img = cv::Mat((int)convertedImage.GetHeight(), (int)convertedImage.GetWidth(), CV_8UC3, (uint8_t*)image.GetBuffer());
                            imshow("View",cv_img);  // display the image in OpenCV image window
                            waitKey(1000);
                            destroyAllWindows();

                            break;
                        }
                    } else if ((key == 'e') || (key == 'E')) {
                        runLoop = false;
                        break;
                    }
                }
            }
        } else {
            // See the documentation of CInstantCamera::CanWaitForFrameTriggerReady() for more information.
            cout << endl
                 << "This sample can only be used with cameras that can be queried whether they are ready to accept the next frame trigger."
                 << endl;
        }
    }
    catch (const GenericException &e) {
        // Error handling.
        cerr << "An exception occurred." << endl << e.GetDescription() << endl;
        exitCode = 1;
    }

    // Releases all pylon resources.
    PylonTerminate();

    return exitCode;
}