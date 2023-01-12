// Namespace for using pylon objects.
using namespace Pylon;

// Namespace for using pylon universal instant camera parameters.
using namespace Basler_UniversalCameraParams;

// Namespace for using cout.
using namespace std;

// Example of a device-specific handler for image events.
class CSampleImageEventHandler : public CBaslerUniversalImageEventHandler
{
public:
    virtual void OnImageGrabbed(CBaslerUniversalInstantCamera & /*camera*/, const CBaslerUniversalGrabResultPtr &ptrGrabResult)
    {
        // Image grabbed successfully?
        if (ptrGrabResult->GrabSucceeded())
        {
            // The chunk data is attached to the grab result and can be accessed anywhere.

            // Generic parameter access:
            // This shows the access via the chunk data node map. This method is available for all grab result types.
            CIntegerParameter chunkTimestamp(ptrGrabResult->GetChunkDataNodeMap(), "ChunkTimestamp");

            // Access the chunk data attached to the result.
            // Before accessing the chunk data, you should check to see
            // if the chunk is readable. When it is readable, the buffer
            // contains the requested chunk data.
            if (chunkTimestamp.IsReadable())
                cout << "OnImageGrabbed: TimeStamp (Result) accessed via node map: " << chunkTimestamp.GetValue() << endl;

            // Native parameter access:
            // When using the device-specific grab results the chunk data can be accessed
            // via the members of the grab result data.
            if (ptrGrabResult->ChunkTimestamp.IsReadable())
                cout << "OnImageGrabbed: TimeStamp (Result) accessed via result member: " << ptrGrabResult->ChunkTimestamp.GetValue() << endl;
        }
    }
};

// Number of images to be grabbed.
static const uint32_t c_countOfImagesToGrab = 5;

int get_chunk()
{
    // The exit code of the sample application.
    int exitCode = 0;

    // Before using any pylon methods, the pylon runtime must be initialized.
    PylonInitialize();

    try
    {
        // Create an instant camera object with the first found camera device.
        CBaslerUniversalInstantCamera camera(CTlFactory::GetInstance().CreateFirstDevice());

        // Print the model name of the camera.
        cout << "Using device " << camera.GetDeviceInfo().GetModelName() << endl;

        // Register an image event handler that accesses the chunk data.
        camera.RegisterImageEventHandler(new CSampleImageEventHandler, RegistrationMode_Append, Cleanup_Delete);

        // Open the camera.
        camera.Open();

        // A GenICam node map is required for accessing chunk data. That's why a small node map is required for each grab result.
        // Creating a lot of node maps can be time consuming.
        // The node maps are usually created dynamically when StartGrabbing() is called.
        // To avoid a delay caused by node map creation in StartGrabbing() you have the option to create
        // a static pool of node maps once before grabbing.
        // camera.StaticChunkNodeMapPoolSize = camera.MaxNumBuffer.GetValue();

        // Enable chunks in general.
        if (!camera.ChunkModeActive.TrySetValue(true))
        {
            throw RUNTIME_EXCEPTION("The camera doesn't support chunk features");
        }

        // Enable time stamp chunks.
        camera.ChunkSelector.SetValue(ChunkSelector_Timestamp);
        camera.ChunkEnable.SetValue(true);

        // Enable frame counter chunks?
        if (camera.ChunkSelector.TrySetValue(ChunkSelector_Framecounter))
        {
            // USB camera devices provide generic counters.
            // An explicit FrameCounter value is not provided by USB camera devices.
            // Enable frame counter chunks.
            camera.ChunkEnable.SetValue(true);
        }

        // Enable CRC checksum chunks.
        camera.ChunkSelector.SetValue(ChunkSelector_PayloadCRC16);
        camera.ChunkEnable.SetValue(true);

        // Start the grabbing of c_countOfImagesToGrab images.
        // The camera device is parameterized with a default configuration which
        // sets up free-running continuous acquisition.
        camera.StartGrabbing(c_countOfImagesToGrab);

        // This smart pointer will receive the grab result data.
        CBaslerUniversalGrabResultPtr ptrGrabResult;

        // Camera.StopGrabbing() is called automatically by the RetrieveResult() method
        // when c_countOfImagesToGrab images have been retrieved.
        while (camera.IsGrabbing())
        {
            // Wait for an image and then retrieve it. A timeout of 5000 ms is used.
            // RetrieveResult calls the image event handler's OnImageGrabbed method.
            camera.RetrieveResult(5000, ptrGrabResult, TimeoutHandling_ThrowException);
            cout << "GrabSucceeded: " << ptrGrabResult->GrabSucceeded() << endl;

            // Image grabbed successfully?
            if (ptrGrabResult->GrabSucceeded())
            {
#ifdef PYLON_WIN_BUILD
                // Display the image
                Pylon::DisplayImage(1, ptrGrabResult);
#endif

                // The result data is automatically filled with received chunk data.
                // (Note:  This is not the case when using the low-level API)
                cout << "SizeX: " << ptrGrabResult->GetWidth() << endl;
                cout << "SizeY: " << ptrGrabResult->GetHeight() << endl;
                const uint8_t *pImageBuffer = (uint8_t *)ptrGrabResult->GetBuffer();
                cout << "Gray value of first pixel: " << (uint32_t)pImageBuffer[0] << endl;

                // Check to see if a buffer containing chunk data has been received.
                if (PayloadType_ChunkData != ptrGrabResult->GetPayloadType())
                {
                    throw RUNTIME_EXCEPTION("Unexpected payload type received.");
                }

                // Since we have activated the CRC Checksum feature, we can check
                // the integrity of the buffer first.
                // Note: Enabling the CRC Checksum feature is not a prerequisite for using
                // chunks. Chunks can also be handled when the CRC Checksum feature is deactivated.
                if (ptrGrabResult->HasCRC() && ptrGrabResult->CheckCRC() == false)
                {
                    throw RUNTIME_EXCEPTION("Image was damaged!");
                }

                // Access the chunk data attached to the result.
                // Before accessing the chunk data, you should check to see
                // if the chunk is readable. When it is readable, the buffer
                // contains the requested chunk data.
                if (ptrGrabResult->ChunkTimestamp.IsReadable())
                {
                    cout << "TimeStamp (Result): " << ptrGrabResult->ChunkTimestamp.GetValue() << endl;
                }

                // USB camera devices provide generic counters. An explicit FrameCounter value is not provided by USB camera devices.
                if (ptrGrabResult->ChunkFramecounter.IsReadable())
                {
                    cout << "FrameCounter (Result): " << ptrGrabResult->ChunkFramecounter.GetValue() << endl;
                }

                cout << endl;
            }
            else
            {
                cout << "Error: " << std::hex << ptrGrabResult->GetErrorCode() << std::dec << " " << ptrGrabResult->GetErrorDescription() << endl;
            }
        }

        // Disable chunk mode.
        camera.ChunkModeActive.SetValue(false);
    }
    catch (const GenericException &e)
    {
        // Error handling.
        cerr << "An exception occurred." << endl
             << e.GetDescription() << endl;
        exitCode = 1;
    }

    // Comment the following two lines to disable waiting on exit.
    cerr << endl
         << "Press enter to exit." << endl;
    while (cin.get() != '\n')
        ;

    // Releases all pylon resources.
    PylonTerminate();

    return exitCode;
}
