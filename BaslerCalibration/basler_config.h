// Include files to use the pylon API.
#include <pylon/PylonIncludes.h>

// Namespace for using pylon objects.
using namespace Pylon;

class BaslerLidarConfiguration : public CConfigurationEventHandler {
public:
    explicit BaslerLidarConfiguration(bool trigger) : trigger(trigger) {}

    /// Apply acquire continuous configuration.
    void ApplyConfiguration(GenApi::INodeMap &nodemap) {
        using namespace GenApi;

        // Disable all trigger types except the trigger type used for triggering the acquisition of
        // frames.
        if(trigger) {
            // Get required enumerations.
            CEnumParameter triggerSelector(nodemap, "TriggerSelector");
            CEnumParameter triggerMode(nodemap, "TriggerMode");

            // Check the available camera trigger mode(s) to select the appropriate one: acquisition start trigger mode
            // (used by older cameras, i.e. for cameras supporting only the legacy image acquisition control mode;
            // do not confuse with acquisition start command) or frame start trigger mode
            // (used by newer cameras, i.e. for cameras using the standard image acquisition control mode;
            // equivalent to the acquisition start trigger mode in the legacy image acquisition control mode).
            String_t triggerName("FrameStart");
            if (!triggerSelector.CanSetValue(triggerName)) {
                triggerName = "AcquisitionStart";
                if (!triggerSelector.CanSetValue(triggerName)) {
                    throw RUNTIME_EXCEPTION(
                            "Could not select trigger. Neither FrameStart nor AcquisitionStart is available.");
                }
            }

            // Get all enumeration entries of trigger selector.
            StringList_t triggerSelectorEntries;
            triggerSelector.GetSettableValues(triggerSelectorEntries);

            // Turn trigger mode off for all trigger selector entries except for the frame trigger given by triggerName.
            for (const auto &triggerSelectorEntry: triggerSelectorEntries) {
                // Set trigger mode to off.
                triggerSelector.SetValue(triggerSelectorEntry);
                if (triggerName == triggerSelectorEntry) {
                    // Activate trigger.
                    triggerMode.SetValue("On");

                    // The trigger source must be set to 'Software'.
                    CEnumParameter(nodemap, "TriggerSource").SetValue("Software");
                } else {
                    triggerMode.TrySetValue("Off");
                }
            }
            // Finally select the frame trigger type
            triggerSelector.SetValue(triggerName);
        }

        //Disable compression mode.
        CConfigurationHelper::DisableCompression(nodemap);

        //Disable GenDC streaming.
        CConfigurationHelper::DisableGenDC(nodemap);

        // Set image component.
        CConfigurationHelper::SelectRangeComponent(nodemap);

        //Set acquisition mode.
        CEnumParameter(nodemap, "AcquisitionMode").SetValue("Continuous");

        // Set AOI
        const int AOI_WIDTH = 800;
        const int AOI_HEIGHT = 800;
        CIntegerParameter(nodemap, "Width").SetValue(AOI_WIDTH, IntegerValueCorrection_Nearest);
        CIntegerParameter(nodemap, "Height").SetValue(AOI_HEIGHT, IntegerValueCorrection_Nearest);
        CCommandParameter(nodemap, "BslCenterX").Execute();
        CCommandParameter(nodemap, "BslCenterY").Execute();

        // Set Exposure
        const float EXPOSURE_TIME_US = 30000;
        CFloatParameter(nodemap, "ExposureTime").SetValue(EXPOSURE_TIME_US);

        // Set Pixel Format
        CEnumParameter(nodemap, "PixelFormat").SetValue("BGR8");

        // Set FPS
        CBooleanParameter(nodemap, "AcquisitionFrameRateEnable").SetValue(true);
        CFloatParameter(nodemap, "AcquisitionFrameRate").SetValue(30.0);
    }


    //Set basic camera settings.
    void OnOpened(CInstantCamera &camera) override {
        try {
            this->ApplyConfiguration(camera.GetNodeMap());
        }
        catch (const GenericException &e) {
            throw RUNTIME_EXCEPTION(
                    "Could not apply configuration. Pylon::GenericException caught in OnOpened method msg=%hs",
                    e.what());
        }
        catch (const std::exception &e) {
            throw RUNTIME_EXCEPTION("Could not apply configuration. std::exception caught in OnOpened method msg=%hs",
                                    e.what());
        }
        catch (...) {
            throw RUNTIME_EXCEPTION("Could not apply configuration. Unknown exception caught in OnOpened method.");
        }
    }

private:
    bool trigger;
};