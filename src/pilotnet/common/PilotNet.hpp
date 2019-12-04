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
// Copyright (c) 2017 NVIDIA Corporation. All rights reserved.
//
// NVIDIA Corporation and its licensors retain all intellectual property and proprietary
// rights in and to this software and related documentation and any modifications thereto.
// Any use, reproduction, disclosure or distribution of this software and related
// documentation without an express license agreement from NVIDIA Corporation is
// strictly prohibited.
//
/////////////////////////////////////////////////////////////////////////////////////////

#ifndef SAMPLES_PILOTNET_HPP__
#define SAMPLES_PILOTNET_HPP__

// Driveworks
#include <dw/Driveworks.h>

// Sample framework
#include <framework/DataPath.hpp>
#include <framework/ProgramArguments.hpp>
#include <framework/SampleFramework.hpp>
#include <framework/MathUtils.hpp>

// Sample framework
#include <framework/DriveWorksSample.hpp>
#include <framework/SimpleCamera.hpp>
#include <framework/SimpleStreamer.hpp>
#include <framework/SimpleFormatConverter.hpp>
#include <framework/SimpleRenderer.hpp>
#include <framework/Checks.hpp>

// using namespace dw_samples::common;

// SAL
#include <dw/sensors/Sensors.h>
#include <dw/sensors/camera/Camera.h>

// Raw
#include <dw/isp/SoftISP.h>

// Renderer
#include <dw/renderer/Renderer.h>

// LaneDetector
#include <dw/lanes/LaneDetector.h>

// FreeSpaceDetector
#include <dw/freespace/FreeSpaceDetector.h>

// DriveNet
#include <dw/object/DriveNet.h>
#include <dw/object/Detector.h>
#include <dw/object/Tracker.h>

// IMAGE
#include <dw/image/FormatConverter.h>
#include <dw/image/ImageStreamer.h>

#include <string>
#include <vector>

// GLFW keys
#include <GLFW/glfw3.h>
#include <framework/WindowGLFW.hpp>

// Screenshot output
#include <lodepng.h>

class PilotNet
{
public:
    PilotNet(uint32_t windowWidth, uint32_t windowHeight, const std::string &inputType);

    virtual bool initializeModules();
    virtual dwStatus runSingleCameraPipeline();
    virtual void releaseModules();    

    size_t getNumClasses() { return classLabels.size(); }

    const std::vector<std::pair<dwBox2D,std::string>>& getResult(uint32_t classIdx);

    // Maximum number of proposals per class object class
    static const uint32_t maxProposalsPerClass = 1000U;

    // Maximum number of objects (clustered proposals) per object class
    static const uint32_t maxClustersPerClass = 400U;

    // rectifier
    void createOutputCUDAImage(dwImageCUDA* output);
    void releaseOutputCUDAImage(dwImageCUDA* output);

protected:
    bool initDriveworks();
    bool initCameras();
    bool initRenderer();
    bool initRectifier();
    bool initRigConfiguration();
    bool initPilotNetWithRig();
    bool initPipeline();
    bool initDNN();

    // DriveNet Detector
    bool initTracker();
    bool initDetector();
    void releaseTracker();
    void releaseDetector();
    void resetTracker();
    void resetDetector();

    void inferDetectorAsync(const dwImageCUDA* rcbImage);
    void inferTrackerAsync(const dwImageCUDA* rcbImage);
    void processResults();
    void drawROI(dwRect roi, const float32_t color[4], dwRenderBufferHandle_t renderBuffer, dwRendererHandle_t renderer);

    void setupRenderer(dwRendererHandle_t &renderer, const dwRect &screenRect, dwContextHandle_t dwSdk);
    void setupLineBuffer(dwRenderBufferHandle_t &lineBuffer, unsigned int maxLines,
                         dwContextHandle_t dwSdk);

    void renderCameraTexture(dwImageStreamerHandle_t streamer, dwRendererHandle_t renderer);
    void drawLaneMarkings(const dwLaneDetection &lanes, float32_t laneWidth,
                          dwRenderBufferHandle_t renderBuffer, dwRendererHandle_t renderer);

    void drawFreeSpaceBoundary(dwFreeSpaceDetection* boundary, dwRenderBufferHandle_t renderBuffer, dwRendererHandle_t renderer);

    void drawLaneMarkingsCustomColor(float32_t laneColors[][4], uint32_t nColors,
                                     const dwLaneDetection &lanes, float32_t laneWidth,
                                     dwRenderBufferHandle_t renderBuffer, dwRendererHandle_t renderer);

    void drawLaneDetectionROI(dwRenderBufferHandle_t renderBuffer, dwRendererHandle_t renderer);
    void drawFreeSpaceDetectionROI(dwRenderBufferHandle_t renderBuffer, dwRendererHandle_t renderer);

    dwStatus runSingleCameraPipelineH264();
    dwStatus runSingleCameraPipelineRaw();

    void runDetector(dwImageCUDA* frame, dwLaneDetectorHandle_t laneDetector, dwFreeSpaceDetectorHandle_t freeSpaceDetector, dwObjectDetectorHandle_t driveNetDetector, dwObjectTrackerHandle_t objectTracker);
    bool createVideoReplay(dwSensorHandle_t &salSensor, float32_t &cameraFrameRate, dwSALHandle_t sal);

    // take screenshot
    void takeInputFrameScreenshot();
    void takeRectifiedFrameScreenshot();
    void takeDriveNetScreenshot();
    void takeLaneNetScreenshot();
    void takeOpenRoadNetScreenshot();
    void takePilotNetScreenshot();
    void onProcessKey(int key);

    // SDK
    // WindowBase *gWindow = nullptr;
    dwContextHandle_t m_sdk;

    // Cameras
    std::string m_inputType;
    dwSALHandle_t m_sal;
    dwSensorHandle_t m_cameraSensor;

    // Renderers
    dwRendererHandle_t m_renderer;
    dwRenderBufferHandle_t m_renderBuffer;

    // Lane detector
    dwLaneDetectorHandle_t m_laneDetector;

    // Freespace detector
    dwFreeSpaceDetectorHandle_t m_freeSpaceDetector;

    // Detector
    dwObjectDetectorHandle_t m_driveNetDetector;
    dwObjectDetectorParams detectorParams{};

    // DriveNet detector
    dwDriveNetHandle_t m_driveNet;
    dwDriveNetParams m_driveNetParams{};
    const dwDriveNetClass *driveNetClasses = nullptr;
    uint32_t numDriveNetClasses = 0;

    // Clustering
    dwObjectClusteringHandle_t *objectClusteringHandles = nullptr;

    // Tracker
    dwObjectTrackerHandle_t m_objectTracker;

    // Colors for rendering bounding boxes
    static const uint32_t MAX_BOX_COLORS = DW_DRIVENET_NUM_CLASSES;
    float32_t boxColors[MAX_BOX_COLORS][4] = {{1.0f, 0.0f, 0.0f, 1.0f},
                                                {0.0f, 1.0f, 0.0f, 1.0f},
                                                {0.0f, 0.0f, 1.0f, 1.0f},
                                                {1.0f, 0.0f, 1.0f, 1.0f},
                                                {1.0f, 0.647f, 0.0f, 1.0f}};

    // Number of proposals per object class
    std::vector<size_t> numProposals;

    // List of proposals per object class
    std::vector<std::vector<dwObject>> objectProposals;

    // Number of clusters per object class
    std::vector<size_t> numClusters;

    // List of clusters per object class
    std::vector<std::vector<dwObject>> objectClusters;

    // Number of merged objects per object class
    std::vector<size_t> numMergedObjects;

    // List of merged objects per object class
    std::vector<std::vector<dwObject>> objectsMerged;

    // Number of tracked objects per object class
    std::vector<size_t> numTrackedObjects;

    // List of tracked objects per object class
    std::vector<std::vector<dwObject>> objectsTracked;

    // Labels of each class
    std::vector<std::string> classLabels;

    // Vector of pairs of boxes and class label ids
    std::vector<std::vector<std::pair<dwBox2D,std::string>>> dnnBoxList;

    // std::unique_ptr<SimpleCamera> camera;
    // std::unique_ptr<SimpleRenderer> simpleRenderer;

    // used for conversion to RGBA and streaming to GL for display
    // std::unique_ptr<GenericSimpleFormatConverter> converterToRGBA;
    // std::unique_ptr<SimpleImageStreamer<dwImageCUDA, dwImageGL>> streamerCUDA2GL;

    // Streamers and converters
    bool m_isRaw;
    bool m_isRig;
    dwImageCUDA m_frameCUDArgba;
    dwImageCUDA m_frameCUDArcb;
    dwImageProperties m_cameraImageProperties;
    dwCameraProperties m_cameraProperties;
    dwSoftISPHandle_t m_softISP;
    dwImageStreamerHandle_t m_streamerInput2CUDA;
    dwImageFormatConverterHandle_t m_converterInput2Rgba;
    dwImageStreamerHandle_t m_streamerCamera2GL;

    // screenshot
    dwImageStreamerHandle_t m_streamerCUDA2CPU = DW_NULL_HANDLE;
    dwImageStreamerHandle_t m_streamerCUDA2GL = DW_NULL_HANDLE;

    bool m_takeScreenshot = false;
    // uint32_t m_screenshotInputFrameCount = 0;
    // uint32_t m_screenshotRectifiedFrameCount = 0;
    // uint32_t m_screenshotDriveNetFrameCount = 0;
    // uint32_t m_screenshotLaneNetFrameCount = 0;
    // uint32_t m_screenshotOpenRoadNetFrameCount = 0;
    // uint32_t m_screenshotPilotNetFrameCount = 0;
    bool m_dump = false;
    std::string m_rigConfigFilename;
    std::string m_videoFilename;

    // execution time
    // typedef std::chrono::high_resolution_clock myclock_t;
    // typedef std::chrono::time_point<myclock_t> timepoint_t;

    // timepoint_t beginPilotNetTime;
    // timepoint_t endDriveNetTime;
    // timepoint_t endLaneNetTime; 
    // timepoint_t endOpenRoadNetTime;
    // timepoint_t endPilotNetTime;

    dwRigConfigurationHandle_t m_rigConfig = DW_NULL_HANDLE;
    dwCameraRigHandle_t m_cameraRig = DW_NULL_HANDLE; 
    dwCalibratedCameraHandle_t m_cameraModelIn = DW_NULL_HANDLE;
    dwCalibratedCameraHandle_t m_cameraModelOut = DW_NULL_HANDLE;

    // to remove fisheye lens distortion
    dwImageCUDA m_rectifiedImage{};
    dwRectifierHandle_t m_rectifier = DW_NULL_HANDLE;
    uint32_t m_cameraCount = 0U;
    float32_t m_fovX = 0.0f;
    float32_t m_fovY = 0.0f;
    uint32_t m_cameraIdx = 0;

    int key = GLFW_KEY_ESCAPE;

    // Screen geometry
    dwRect m_screenRectangle;
    uint32_t m_windowWidth;
    uint32_t m_windowHeight;

    uint32_t m_cameraWidth;
    uint32_t m_cameraHeight;

    float32_t m_threshold;

    float32_t m_maxDistance;

    // Cuda stream
    cudaStream_t m_cudaStream;

#ifdef DW_USE_NVMEDIA
    dwImageFormatConverterHandle_t m_converterNvMYuv2rgba;
    dwImageStreamerHandle_t m_streamerNvMedia2CUDA;
    dwImageNvMedia m_frameNVMrgba;
#endif

};

#endif // SAMPLES_PILOTNET_HPP__
