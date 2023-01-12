// Namespace for using pylon objects.
using namespace Pylon;

// Namespace for using pylon universal instant camera parameters.
using namespace Basler_UniversalCameraParams;

// Namespace for using cout.
using namespace std;

int run()
{
    // The exit code of the sample application.
    int exitCode = 0;

    // Before using any pylon methods, the pylon runtime must be initialized.
    PylonInitialize();

    try
    {
        // Create an instant camera object with the first found camera device.
        CBaslerUniversalInstantCamera camera(CTlFactory::GetInstance().CreateFirstDevice());

        // Open the camera for accessing the parameters.
        camera.Open();

        // Get camera device information.
        cout << "Camera Device Information" << endl
             << "=========================" << endl;
        cout << "Vendor           : "
             << camera.DeviceVendorName.GetValue() << endl;
        cout << "Model            : "
             << camera.DeviceModelName.GetValue() << endl;
        cout << "Firmware version : "
             << camera.DeviceFirmwareVersion.GetValue() << endl
             << endl;

        // Camera settings.
        cout << "Camera Device Settings" << endl
             << "======================" << endl;

        // Set the AOI:

        // On some cameras, the offsets are read-only.
        // Therefore, we must use "Try" functions that only perform the action
        // when parameters are writable. Otherwise, we would get an exception.
        camera.OffsetX.TrySetToMinimum();
        camera.OffsetY.TrySetToMinimum();

        // Some properties have restrictions.
        // We use API functions that automatically perform value corrections.
        // Alternatively, you can use GetInc() / GetMin() / GetMax() to make sure you set a valid value.
        camera.Width.SetValue(202, IntegerValueCorrection_Nearest);
        camera.Height.SetValue(101, IntegerValueCorrection_Nearest);

        cout << "OffsetX          : " << camera.OffsetX.GetValue() << endl;
        cout << "OffsetY          : " << camera.OffsetY.GetValue() << endl;
        cout << "Width            : " << camera.Width.GetValue() << endl;
        cout << "Height           : " << camera.Height.GetValue() << endl;

        // Remember the current pixel format.
        PixelFormatEnums oldPixelFormat = camera.PixelFormat.GetValue();
        cout << "Old PixelFormat  : " << camera.PixelFormat.ToString() << " (" << oldPixelFormat << ")" << endl;

        // Set pixel format to Mono8 if available.
        if (camera.PixelFormat.CanSetValue(PixelFormat_Mono8))
        {
            camera.PixelFormat.SetValue(PixelFormat_Mono8);
            cout << "New PixelFormat  : " << camera.PixelFormat.ToString() << " (" << camera.PixelFormat.GetValue() << ")" << endl;
        }

        // Set the measuring location to sensor
        camera.DeviceTemperatureSelector.SetValue(DeviceTemperatureSelector_FpgaCore);
        // Get the current temperature
        auto e = camera.DeviceTemperature.GetValue();
        cout << "Current Temp     : " << e << endl;
        // Get the maximum temperature the camera reached during operation
        double temperatureMax = camera.BslTemperatureMax.GetValue();
        cout << "Max Temp         : " << temperatureMax << endl;
        // Determine how often the temperature state changed to Error
        int64_t i = camera.BslTemperatureStatusErrorCount.GetValue();
        cout << "# Temp Errors    : " << i << endl;

        // Set the new gain to 50% ->  Min + ((Max-Min) / 2).
        //
        // Note: Some newer camera models may have auto functions enabled.
        //       To be able to set the gain value to a specific value
        //       the Gain Auto function must be disabled first.
        // Access the enumeration type node GainAuto.
        // We use a "Try" function that only performs the action if the parameter is writable.
        camera.GainAuto.TrySetValue(GainAuto_Off);

        if (camera.GetSfncVersion() >= Sfnc_2_0_0) // Cameras based on SFNC 2.0 or later, e.g., USB cameras
        {
            camera.Gain.SetValuePercentOfRange(50.0);
            cout << "Gain (50%)       : " << camera.Gain.GetValue() << " (Min: " << camera.Gain.GetMin() << "; Max: " << camera.Gain.GetMax() << ")" << endl;
        }
        else
        {
            camera.GainRaw.SetValuePercentOfRange(50.0);
            cout << "Gain (50%)       : " << camera.GainRaw.GetValue() << " (Min: " << camera.GainRaw.GetMin() << "; Max: " << camera.GainRaw.GetMax() << "; Inc: " << camera.GainRaw.GetInc() << ")" << endl;
        }

        // Restore the old pixel format.
        camera.PixelFormat.SetValue(oldPixelFormat);

        // Close the camera.
        camera.Close();
    }
    catch (const GenericException &e)
    {
        // Error handling.
        cerr << "An exception occurred." << endl
             << e.GetDescription() << endl;
        exitCode = 1;
    }

    // Releases all pylon resources.
    PylonTerminate();

    return exitCode;
}
