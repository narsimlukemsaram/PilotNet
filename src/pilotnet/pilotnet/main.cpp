/////////////////////////////////////////////////////////////////////////////////////////
// This code contains NVIDIA Confidential Information and is disclosed
// under the Mutual Non-Disclosure Agreement.
//
// Notice
// ALL NVIDIA DESIGN SPECIFICATIONS AND CODE ("MATERIALS") ARE PROVIDED "AS IS" NVIDIA MAKES
// NO REPRESENTATIONS, WARRANTIES, EXPRESSED, IMPLIED, STATUTORY, OR OTHERWISE WITH RESPECT TO
// THE MATERIALS, AND EXPRESSLY DISCLAIMS ANY IMPLIED WARRANTIES OF NONINFRINGEMENT,
// MERCHANTABILITY, OR FITNESS FOR A PARTICULAR PURPOSE.
//
// NVIDIA Corporation assumes no responsibility for the consequences of use of such
// information or for any infringement of patents or other rights of third parties that may
// result from its use. No license is granted by implication or otherwise under any patent
// or patent rights of NVIDIA Corporation. No third party distribution is allowed unless
// expressly authorized by NVIDIA.  Details are subject to change without notice.
// This code supersedes and replaces all information previously supplied.
// NVIDIA Corporation products are not authorized for use as critical
// components in life support devices or systems without express written approval of
// NVIDIA Corporation.
//
// Copyright (c) 2015-2016 NVIDIA Corporation. All rights reserved.
//
// NVIDIA Corporation and its licensors retain all intellectual property and proprietary
// rights in and to this software and related documentation and any modifications thereto.
// Any use, reproduction, disclosure or distribution of this software and related
// documentation without an express license agreement from NVIDIA Corporation is
// strictly prohibited.
//
/////////////////////////////////////////////////////////////////////////////////////////
#define _CRT_SECURE_NO_WARNINGS

#include <thread>

// Sample
#include <pilotnet/common/PilotNet.hpp>

// Calibrated camera
#include <dw/rigconfiguration/RigConfiguration.h>
#include <dw/rigconfiguration/Camera.h>

// GLFW keys
#include <framework/WindowGLFW.hpp>

// Driveworks
#include <dw/Driveworks.h>

class PilotNetCalibratedCamera : public PilotNet
{
public:
    PilotNetCalibratedCamera(uint32_t windowWidth,
                            uint32_t windowHeight,
                            const std::string &inputType)
        : PilotNet(windowWidth, windowHeight, inputType)
    {}
    virtual bool initializeModules() override
    {
        if (!PilotNet::initDriveworks()) return false;
        if (!PilotNet::initCameras()) return false;
        if (!PilotNet::initRenderer()) return false;
        if (!PilotNet::initRigConfiguration()) return false;
        if (!PilotNet::initPilotNetWithRig()) return false;
        if (!PilotNet::initRectifier()) return false;
        if (!PilotNet::initPipeline()) return false;
        if (!PilotNet::initDetector()) return false;  
        if (!PilotNet::initTracker()) return false;      

        // start cameras as late as possible, so that all initialization routines are finished before
        gRun = gRun && dwSensor_start(m_cameraSensor) == DW_SUCCESS;

        return true;
    }

    virtual dwStatus runSingleCameraPipeline() override    
    {
        dwStatus status = DW_SUCCESS;
        if (m_isRaw) {
            status = PilotNet::runSingleCameraPipelineRaw();
        } else {
            status = PilotNet::runSingleCameraPipelineH264();
        }

        if (status == DW_END_OF_STREAM) {
            std::cout << "Camera reached end of stream" << std::endl;
            dwSensor_reset(m_cameraSensor);
            gRun = false;
        }
        else if (status != DW_SUCCESS) {
            gRun = false;
        }

        return status;
    }        

    virtual void releaseModules() override
    {
        PilotNet::releaseModules();
    }

    virtual void onProcessKey(int key)
    {
        std::cout << "main()=>onProcessKey " << "\n"; 

        // take screenshot
        if (key == GLFW_KEY_S) {
            m_takeScreenshot = true;
            std::cout << "main()=>take screenshot " << "\n";  
        }
    }
};

//#######################################################################################
int main(int argc, const char **argv)
{
    const ProgramArguments arguments = ProgramArguments({
#ifdef DW_USE_NVMEDIA
            ProgramArguments::Option_t("camera-type", "ar0231-rccb-ssc"),
            ProgramArguments::Option_t("csi-port", "ab"),
            ProgramArguments::Option_t("camera-index", "0"),
            ProgramArguments::Option_t("slave", "0"),
            ProgramArguments::Option_t("input-type", "camera"),
#endif
            ProgramArguments::Option_t("video", (DataPath::get() + "/samples/pilotnet/PilotNet_Video_1.h264").c_str()),
            ProgramArguments::Option_t("rig", (DataPath::get() + "/samples/pilotnet/rig.xml").c_str()),
            ProgramArguments::Option_t("stopFrame", "0"),
            ProgramArguments::Option_t("threshold", "0.3"),
            ProgramArguments::Option_t("maxDistance", "50.0"),
            ProgramArguments::Option_t("fov", "60"),
            ProgramArguments::Option_t("fovX", "60.6"),
            ProgramArguments::Option_t("fovY", "36.1"),
            ProgramArguments::Option_t("cameraIdx", "0"),
            ProgramArguments::Option_t("dumpToFile", "0"),
        });

    // Default window width and height
    uint32_t windowWidth = 960;
    uint32_t windowHeight = 576;

    // init framework
    initSampleApp(argc, argv, &arguments, NULL, windowWidth, windowHeight);

    std::string inputType = "video";
#ifdef DW_USE_NVMEDIA
    inputType = gArguments.get("input-type");
#endif

    PilotNetCalibratedCamera pilotNet(windowWidth, windowHeight, inputType);

    // init driveworks
    if (!pilotNet.initializeModules())
    {
        std::cerr << "Cannot initialize DW subsystems" << std::endl;
        gRun = false;
    }

    typedef std::chrono::high_resolution_clock myclock_t;
    typedef std::chrono::time_point<myclock_t> timepoint_t;
    timepoint_t lastUpdateTime = myclock_t::now();

    // int key = GLFW_KEY_ESCAPE;

    // main loop
    while (gRun && !gWindow->shouldClose()) {
        std::this_thread::yield();

        bool processImage = true;

        // run with at most 30FPS
        std::chrono::milliseconds timeSinceUpdate = std::chrono::duration_cast<std::chrono::milliseconds>(myclock_t::now() - lastUpdateTime);
        if (timeSinceUpdate < std::chrono::milliseconds(33)) //33
            processImage = false;

        if (processImage) {

            lastUpdateTime = myclock_t::now();

            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            pilotNet.runSingleCameraPipeline();

            // pilotNet.processKey(key);

            gWindow->swapBuffers();
        }
    }

    // Release modules and driveworks
    pilotNet.releaseModules();

    // release framework
    releaseSampleApp();

    return 0;
}
