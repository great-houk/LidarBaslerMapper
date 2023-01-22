// Grab_UsingGrabLoopThread.cpp
/*
    Note: Before getting started, Basler recommends reading the "Programmer's Guide" topic
    in the pylon C++ API documentation delivered with pylon.
    If you are upgrading to a higher major version of pylon, Basler also
    strongly recommends reading the "Migrating from Previous Versions" topic in the pylon C++ API documentation.

    This sample illustrates how to grab and process images using the grab loop thread
    provided by the Instant Camera class.
*/

// Include files to use the pylon API.
#include <pylon/PylonIncludes.h>

// Include files used by samples.
#include "ConfigurationEventPrinter.h"

// Include Config File
#include "../BaslerCalibration/basler_config.h"

// Namespace for using pylon objects.
using namespace Pylon;

// Namespace for using cout.
using namespace std;

int capture_basler() {
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
        camera.RegisterConfiguration(new BaslerLidarConfiguration(false), RegistrationMode_ReplaceAll, Cleanup_Delete);

        // For demonstration purposes only, add a sample configuration event handler to print out information
        // about camera use.
        camera.RegisterConfiguration(new CConfigurationEventPrinter, RegistrationMode_Append, Cleanup_Delete);

        // Open the camera device.
        camera.Open();

        // Capture one frame
        CGrabResultPtr ptrGrabResult;

        // Try to get a grab result.
        cout << endl << "Waiting for an image to be grabbed." << endl;
        try {
            camera.GrabOne(1000, ptrGrabResult);
        }
        catch (const GenericException &e) {

            cerr << "Could not grab an image: " << endl
                 << e.GetDescription() << endl;
        }
        cout << endl << "Grabbed an image!" << endl << endl;

        // Save image to file
        if (ptrGrabResult && ptrGrabResult->GrabSucceeded() && ptrGrabResult.IsValid()) {
            cout << "Saving Image..." << endl;
            // Create a pylon image.
            CPylonImage image;

            // Initializes the image object with the buffer from the grab result.
            // This prevents the reuse of the buffer for grabbing as long as it is
            // not released.
            // Please note that this is not relevant for this example because the
            // camera object has been destroyed already.
            image.AttachGrabResultBuffer(ptrGrabResult);

            // Save image to a file
            image.Save(ImageFileFormat_Png, MAIN_DIR"/calib/basler_calib.png");
        } else {
            cerr << "Image grab isn't valid." << endl;
            exitCode = -1;
        }
        // Now the grab result can be released. The grab result buffer is now
        // only held by the pylon image.
//        ptrGrabResult.Release();
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