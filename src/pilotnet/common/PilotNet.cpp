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

#include "PilotNet.hpp"

#include <framework/Checks.hpp>

#include <iostream>

// Driveworks
#include <dw/Driveworks.h>

#include <lodepng.h>

//#######################################################################################
PilotNet::PilotNet(uint32_t windowWidth, uint32_t windowHeight, const std::string &inputType)
    : m_sdk(DW_NULL_HANDLE)
    , m_inputType(inputType)
    , m_sal(DW_NULL_HANDLE)
    , m_cameraSensor(DW_NULL_HANDLE)
    , m_renderer(DW_NULL_HANDLE)
    , m_renderBuffer(DW_NULL_HANDLE)
    , m_laneDetector(DW_NULL_HANDLE)
    , m_freeSpaceDetector(DW_NULL_HANDLE)
    , m_driveNetDetector(DW_NULL_HANDLE)
    , m_driveNet(DW_NULL_HANDLE)
    , m_driveNetParams{}
    , m_objectTracker(DW_NULL_HANDLE)
    , m_isRaw(false)
    , m_frameCUDArgba{}
    , m_frameCUDArcb{}
    , m_cameraImageProperties{}
    , m_cameraProperties{}
    , m_softISP(DW_NULL_HANDLE)
    , m_streamerInput2CUDA(DW_NULL_HANDLE)
    , m_converterInput2Rgba(DW_NULL_HANDLE)
    , m_streamerCamera2GL(DW_NULL_HANDLE)
    , m_streamerCUDA2CPU(DW_NULL_HANDLE)
    , m_streamerCUDA2GL(DW_NULL_HANDLE)
    , m_rigConfig(DW_NULL_HANDLE)
    , m_cameraRig(DW_NULL_HANDLE)    
    , m_cameraModelIn(DW_NULL_HANDLE)
    , m_cameraModelOut(DW_NULL_HANDLE)
    , m_rectifiedImage{}
    , m_rectifier(DW_NULL_HANDLE)
    , m_fovX(0.0f)
    , m_fovY(0.0f)
    , m_screenRectangle{}
    , m_windowWidth(windowWidth)
    , m_windowHeight(windowHeight)
    , m_cameraWidth(0)
    , m_cameraHeight(0)
    , m_threshold(0.0f)
    , m_cudaStream(0)
    #ifdef DW_USE_NVMEDIA
    , m_converterNvMYuv2rgba(DW_NULL_HANDLE)
    , m_streamerNvMedia2CUDA(DW_NULL_HANDLE)
    , m_frameNVMrgba{}
    // , m_screenshotInputFrameCount(0)
    // , m_screenshotRectifiedFrameCount(0)
    // , m_screenshotDriveNetFrameCount(0)
    // , m_screenshotLaneNetFrameCount(0)
    // , m_screenshotOpenRoadNetFrameCount(0)
    // , m_screenshotPilotNetFrameCount(0)
    #endif
{
    m_rigConfigFilename = gArguments.get("rig");
    m_videoFilename     = gArguments.get("video");
    m_fovX              = DEG2RAD(atof(gArguments.get("fovX").c_str()));
    m_fovY              = DEG2RAD(atof(gArguments.get("fovY").c_str()));
    m_cameraIdx         = atoi(gArguments.get("cameraIdx").c_str());
    m_dump              = gArguments.get("dumpToFile").compare("1") == 0;
    m_takeScreenshot    = false;
}

//#######################################################################################
bool PilotNet::initializeModules()
{
    // beginPilotNetTime = myclock_t::now();

    // start cameras as late as possible, so that all initialization routines are finished before
    // gRun = gRun && dwSensor_start(m_cameraSensor) == DW_SUCCESS;

    return true;
}

//#######################################################################################
bool PilotNet::initDriveworks()
{
    // create a Logger to log to console
    // we keep the ownership of the logger at the application level
    dwLogger_initialize(getConsoleLoggerCallback(true));
    dwLogger_setLogLevel(DW_LOG_DEBUG);

    // instantiate Driveworks SDK context
    dwContextParameters sdkParams{};

    std::string path = DataPath::get();
    sdkParams.dataPath = path.c_str();

#ifdef VIBRANTE
    sdkParams.eglDisplay = gWindow->getEGLDisplay();
#endif

    return dwInitialize(&m_sdk, DW_VERSION, &sdkParams) == DW_SUCCESS;
}

//#######################################################################################
bool PilotNet::initCameras()
{
    dwStatus result = DW_FAILURE;

    // create sensor abstraction layer
    result = dwSAL_initialize(&m_sal, m_sdk);
    if (result != DW_SUCCESS) {
        std::cerr << "Cannot init sal: " << dwGetStatusName(result) << std::endl;
        return false;
    }

    // create GMSL Camera interface
    float32_t cameraFramerate = 0.0f;

    if(!createVideoReplay(m_cameraSensor, cameraFramerate, m_sal))
        return false;

    std::cout << "Camera image with " << m_cameraWidth << "x" << m_cameraHeight << " at "
              << cameraFramerate << " FPS" << std::endl;

    return true;
}

//#######################################################################################
bool PilotNet::initRenderer()
{
    // init renderer
    m_screenRectangle.height = gWindow->height();
    m_screenRectangle.width = gWindow->width();
    m_screenRectangle.x = 0;
    m_screenRectangle.y = 0;

    unsigned int maxLines = 20000;
    setupRenderer(m_renderer, m_screenRectangle, m_sdk);
    setupLineBuffer(m_renderBuffer, maxLines, m_sdk);

    return true;
}

//#######################################################################################
bool PilotNet::initRigConfiguration()
{
        dwStatus result = DW_SUCCESS;

        // load vehicle configuration
        result = dwRigConfiguration_initializeFromFile(&m_rigConfig, m_sdk, m_rigConfigFilename.c_str());
        if (result != DW_SUCCESS) {
            std::cerr << "Error dwRigConfiguration_initialize: " << dwGetStatusName(result) << std::endl;
            return false;
        }

        result = dwCameraRig_initializeFromConfig(&m_cameraRig, &m_cameraCount,
                                                  &m_cameraModelIn,
                                                  1U, m_sdk, m_rigConfig);
        if (result != DW_SUCCESS) {
            std::cerr << "Error dwCameraRig_initializeFromConfig: " << dwGetStatusName(result) << std::endl;
            return false;
        }

        if ((m_cameraIdx >= m_cameraCount) || (m_cameraIdx < 0)) {
            logError("Invalid cameraIdx. Check your rig.xml file.");
            return false;
        }

        // initialize output camera model as simple pinhole
        dwPinholeCameraConfig cameraConf = {};
        cameraConf.distortion[0] = 0.f;
        cameraConf.distortion[1] = 0.f;
        cameraConf.distortion[2] = 0.f;

        cameraConf.u0 = static_cast<float32_t>(m_cameraWidth/2);
        cameraConf.v0 = static_cast<float32_t>(m_cameraHeight/2);
        cameraConf.width = m_cameraWidth;
        cameraConf.height = m_cameraHeight;

        dwVector2f focal = focalFromFOV({m_fovX, m_fovY}, {cameraConf.width, cameraConf.height});
        cameraConf.focalX = focal.x;
        cameraConf.focalY = focal.y;

        result = dwCalibratedCamera_initializePinhole(&m_cameraModelOut, m_sdk, &cameraConf);

        if(result != DW_SUCCESS) {
            std::cerr << "Cannot initialize pinhole camera: " << dwGetStatusName(result) << std::endl;
            return false;
        }

        return true;
}

//#######################################################################################
bool PilotNet::initPilotNetWithRig()
{
        dwStatus res = DW_FAILURE;
        float32_t maxDistance = 50.0f;        

        res = dwLaneDetector_initializeLaneNet(&m_laneDetector,
                                           m_cameraWidth, m_cameraHeight,
                                           m_sdk);
        if (res != DW_SUCCESS)
        {
            std::cerr << "Cannot initialize LaneNet: " << dwGetStatusName(res) << std::endl;
            return false;
        }

        m_threshold = 0.3f;
        std::string inputThreshold = gArguments.get("threshold");
        if(inputThreshold!="0.3"){
            try{
            m_threshold = std::stof(inputThreshold);
            } catch(...) {
                std::cerr << "Given threshold can't be parsed" << std::endl;
                return false;
           }
        }   

        res = dwLaneDetectorLaneNet_setDetectionThreshold(m_threshold, m_laneDetector);
        if (res != DW_SUCCESS)
        {
            std::cerr << "Cannot set PilotNet threshold: " << dwGetStatusName(res) << std::endl;
            return false;
        }

        dwTransformation transformation{};
        res = dwRigConfiguration_getSensorToRigTransformation(&transformation, 0, m_rigConfig);
        if (res != DW_SUCCESS) // only compute free space boundary in image space
        {
            std::cerr << "Cannot parse rig configuration: " << dwGetStatusName(res) <<std::endl;
            std::cerr << "Compute free space boundary in image space only." <<std::endl;
            res = dwFreeSpaceDetector_initializeFreeSpaceNet(&m_freeSpaceDetector,
                                                            m_cameraWidth, m_cameraHeight,
                                                            m_cudaStream,
                                                            m_sdk);
        }
        else // compute free space boundary in image and vehicle coordinates
        {
            std::string maxDistanceStr = gArguments.get("maxDistance");
            if(maxDistanceStr!="50.0") {
                try{
                    maxDistance = std::stof(maxDistanceStr);
                    if (maxDistance < 0.0f) {
                        std::cerr << "maxDistance cannot be negative." << std::endl;
                        return false;
                    }
                } catch(...) {
                    std::cerr << "Given maxDistance can't be parsed" << std::endl;
                    return false;
                }
            }
            res = dwFreeSpaceDetector_initializeCalibratedFreeSpaceNet(&m_freeSpaceDetector,
                                                                    m_cameraWidth, m_cameraHeight,
                                                                    m_cudaStream, transformation, maxDistance,
                                                                    m_cameraModelIn, m_sdk);
            if (res != DW_SUCCESS)
            {
                std::cerr << "Cannot initialize FreeSpaceNet: " << dwGetStatusName(res) << std::endl;
                return false;
            }
        }

        res = dwLaneDetector_setCameraHandle(m_cameraModelIn, m_laneDetector);
        if (res != DW_SUCCESS)
        {
            std::cerr << "Cannot initialize lane detector calibrated camera: "
                      << dwGetStatusName(res) << std::endl;
            return false;
        }

        res = dwLaneDetector_setCameraExtrinsics(transformation, m_laneDetector);
        if (res != DW_SUCCESS)
        {
            std::cerr << "Cannot initialize lane detector camera extrinsics: "
                      << dwGetStatusName(res) << std::endl;
            return false;
        }

        res = dwLaneDetector_setMaxLaneDistance(maxDistance, m_laneDetector);
        if (res != DW_SUCCESS)
        {
            std::cerr << "Cannot set lane detector maximum detection distance in meter: "
                      << dwGetStatusName(res) << std::endl;
            return false;
        }

        float32_t fov;
        res = dwCalibratedCamera_getHorizontalFOV(&fov, m_cameraModelIn);
        if (res != DW_SUCCESS)
        {
            std::cerr << "Cannot get camera horizontal FOV: "
                      << dwGetStatusName(res) << std::endl;
            return false;
        }

        res =  dwLaneDetectorLaneNet_setHorizontalFOV(RAD2DEG(fov), m_laneDetector);
        if (res != DW_SUCCESS)
        {
            std::cerr << "Cannot set camera horizontal FOV: "
                      << dwGetStatusName(res) << std::endl;
            return false;
        }

        if(m_isRaw) {
            // this is in case the network requires a different tonemapper than the default one
            dwDNNMetaData metaData;
            dwLaneDetectorLaneNet_getDNNMetaData(&metaData, m_laneDetector);
            dwSoftISP_setTonemapType(metaData.tonemapType, m_softISP);
            dwFreeSpaceDetector_getDNNMetaData(&metaData, m_freeSpaceDetector);
            dwSoftISP_setTonemapType(metaData.tonemapType, m_softISP);
        }        

        return true;
}

//#######################################################################################
bool PilotNet::initRectifier()
{
    dwStatus status = dwRectifier_initialize(&m_rectifier, m_cameraModelIn,
                                             m_cameraModelOut, m_sdk);

    if (status != DW_SUCCESS) {
        std::cerr << "Cannot initialize rectifier: " << dwGetStatusName(status) << std::endl;
        return false;
    }

    createOutputCUDAImage(&m_rectifiedImage);
    return true;
}

//#######################################################################################
void PilotNet::createOutputCUDAImage(dwImageCUDA* output)
{
    cudaMallocPitch(&output->dptr[0], &output->pitch[0], m_cameraWidth*4, m_cameraHeight);
    output->layout = DW_IMAGE_CUDA_PITCH;
    output->prop.height = static_cast<int32_t>(m_cameraHeight);
    output->prop.width = static_cast<int32_t>(m_cameraWidth);
    output->prop.planeCount = 1;
    output->prop.pxlFormat = DW_IMAGE_RGBA;
    output->prop.pxlType = DW_TYPE_UINT8;
    output->prop.type = DW_IMAGE_CUDA;
}

//#######################################################################################
bool PilotNet::initPipeline()
{
    dwStatus status = DW_FAILURE;

#ifdef DW_USE_NVMEDIA
    // NvMedia yuv -> rgba format converter
    dwImageProperties cameraImageProperties = m_cameraImageProperties;
    cameraImageProperties.type = DW_IMAGE_NVMEDIA;

    dwImageFormatConverter_initialize(&m_converterNvMYuv2rgba, cameraImageProperties.type, m_sdk);

    // NvMedia -> CUDA image streamer
    status = dwImageStreamer_initialize(&m_streamerNvMedia2CUDA, &cameraImageProperties,
                                        DW_IMAGE_CUDA, m_sdk);
    if (status != DW_SUCCESS) {
        std::cerr << "Cannot init image streamer: " << dwGetStatusName(status) << std::endl;
        return false;
    }
#endif

    if(m_isRaw){
        dwImageProperties cameraImageProperties = m_cameraImageProperties;
        dwCameraProperties cameraProperties = m_cameraProperties;

        // Raw pipeline
        dwImageProperties rccbImageProperties;
        dwSoftISPParams softISPParams;
        dwSoftISP_initParamsFromCamera(&softISPParams, cameraProperties);
        status = dwSoftISP_initialize(&m_softISP, softISPParams, m_sdk);

        status = status != DW_SUCCESS ? status : dwSoftISP_setCUDAStream(m_cudaStream, m_softISP);
        status = status != DW_SUCCESS ? status : dwSoftISP_setDemosaicMethod(DW_SOFT_ISP_DEMOSAIC_METHOD_INTERPOLATION, m_softISP);
        status = status != DW_SUCCESS ? status : dwSoftISP_getDemosaicImageProperties(&rccbImageProperties, m_softISP);
        if (status != DW_SUCCESS) {
            std::cerr << "Cannot initialize raw pipeline: " << dwGetStatusName(status) << std::endl;
            return false;
        }

        // Input -> CUDA streamer
        dwImageStreamer_initialize(&m_streamerInput2CUDA, &cameraImageProperties, DW_IMAGE_CUDA, m_sdk);

        // Input -> RGBA format converter
        dwImageProperties displayImageProperties = rccbImageProperties;
        displayImageProperties.pxlFormat = DW_IMAGE_RGBA;
        displayImageProperties.pxlType = DW_TYPE_UINT8;
        displayImageProperties.planeCount = 1;
        status = dwImageFormatConverter_initialize(&m_converterInput2Rgba, rccbImageProperties.type, m_sdk);
        if (status != DW_SUCCESS) {
            std::cerr << "Cannot initialize input -> rgba format converter: " << dwGetStatusName(status) << std::endl;
            return false;
        }

        // Setup RCB image
        m_frameCUDArcb.prop = rccbImageProperties;
        m_frameCUDArcb.layout = DW_IMAGE_CUDA_PITCH;
        cudaMallocPitch(&m_frameCUDArcb.dptr[0], &m_frameCUDArcb.pitch[0], rccbImageProperties.width * dwSizeOf(rccbImageProperties.pxlType),
                rccbImageProperties.height * rccbImageProperties.planeCount);
        m_frameCUDArcb.pitch[1] = m_frameCUDArcb.pitch[2] = m_frameCUDArcb.pitch[0];
        m_frameCUDArcb.dptr[1] = reinterpret_cast<uint8_t*>(m_frameCUDArcb.dptr[0]) + rccbImageProperties.height * m_frameCUDArcb.pitch[0];
        m_frameCUDArcb.dptr[2] = reinterpret_cast<uint8_t*>(m_frameCUDArcb.dptr[1]) + rccbImageProperties.height * m_frameCUDArcb.pitch[1];

        dwSoftISP_bindDemosaicOutput(&m_frameCUDArcb, m_softISP);

        // Camera -> GL image streamer
        status = dwImageStreamer_initialize(&m_streamerCamera2GL, &displayImageProperties, DW_IMAGE_GL, m_sdk);
        if (status != DW_SUCCESS) {
            std::cerr << "Cannot init GL streamer: " << dwGetStatusName(status) << std::endl;
            return false;
        }

        // Setup RGBA CUDA image
        {
            void *dptr = nullptr;
            size_t pitch = 0;
            cudaMallocPitch(&dptr, &pitch, rccbImageProperties.width * 4, rccbImageProperties.height);
            dwImageCUDA_setFromPitch(&m_frameCUDArgba, dptr, rccbImageProperties.width, rccbImageProperties.height,
                                     pitch, DW_IMAGE_RGBA);
            dwSoftISP_bindTonemapOutput(&m_frameCUDArgba, m_softISP);
        }
#ifdef DW_USE_NVMEDIA
        // Setup RGBA NvMedia image
        {
            dwImageProperties properties = rccbImageProperties;
            properties.type = DW_IMAGE_NVMEDIA;
            properties.pxlFormat = DW_IMAGE_RGBA;
            properties.pxlType = DW_TYPE_UINT8;
            properties.planeCount = 1;
            dwImageNvMedia_create(&m_frameNVMrgba, &properties, m_sdk);
        }
#endif
        // Set camera width and height for DNN
        m_cameraWidth = rccbImageProperties.width;
        m_cameraHeight = rccbImageProperties.height;
    }
    else{
        dwImageProperties cameraImageProperties = m_cameraImageProperties;

        // Input -> RGBA format converter
        dwImageProperties displayImageProperties = cameraImageProperties;
        displayImageProperties.pxlFormat = DW_IMAGE_RGBA;
        displayImageProperties.pxlType = DW_TYPE_UINT8;
        displayImageProperties.planeCount = 1;
        displayImageProperties.type = DW_IMAGE_CUDA;
        cameraImageProperties.type = DW_IMAGE_CUDA;

        status = dwImageFormatConverter_initialize(&m_converterInput2Rgba, cameraImageProperties.type, m_sdk);

        if (status != DW_SUCCESS) {
            std::cerr << "Cannot initialize input -> rgba format converter: " << dwGetStatusName(status) << std::endl;
            return false;
        }

        // Camera -> GL image streamer
#ifdef DW_USE_NVMEDIA
        displayImageProperties.type = DW_IMAGE_NVMEDIA;
#endif
        status = dwImageStreamer_initialize(&m_streamerCamera2GL, &displayImageProperties, DW_IMAGE_GL, m_sdk);

        if (status != DW_SUCCESS) {
            std::cerr << "Cannot init GL streamer: " << dwGetStatusName(status) << std::endl;
            return false;
        }

        // screenshot
        status = dwImageStreamer_initialize(&m_streamerCUDA2CPU, &displayImageProperties, DW_IMAGE_CPU, m_sdk);
        if (status != DW_SUCCESS) {
            std::cerr << "Cannot init CUDA2CPU image streamer: " << dwGetStatusName(status) << std::endl;
            return false;
        }

        // Setup RGBA CUDA image
        {
            void *dptr = nullptr;
            size_t pitch = 0;
            cudaMallocPitch(&dptr, &pitch, m_cameraWidth * 4, m_cameraHeight);
            dwImageCUDA_setFromPitch(&m_frameCUDArgba, dptr, m_cameraWidth, m_cameraHeight,
                                     pitch, DW_IMAGE_RGBA);
        }

#ifdef DW_USE_NVMEDIA
        // Setup RGBA NvMedia image
        {
            dwImageProperties properties = cameraImageProperties;
            properties.type = DW_IMAGE_NVMEDIA;
            properties.pxlFormat = DW_IMAGE_RGBA;
            properties.pxlType = DW_TYPE_UINT8;
            properties.planeCount = 1;
            properties.width = m_cameraWidth;
            properties.height = m_cameraHeight;
            dwImageNvMedia_create(&m_frameNVMrgba, &properties, m_sdk);
        }
#endif
        // Set camera width and height for DNN
        m_cameraWidth = cameraImageProperties.width;
        m_cameraHeight = cameraImageProperties.height;
    }

    return true;
}

//#######################################################################################
bool PilotNet::initDetector()
{
    // Initialize DriveNet network
    CHECK_DW_ERROR(dwDriveNet_initDefaultParams(&m_driveNetParams));
    // Set up max number of proposals and clusters
    m_driveNetParams.maxClustersPerClass = maxClustersPerClass;
    m_driveNetParams.maxProposalsPerClass = maxProposalsPerClass;
    // Set batch size to 2 for foveal
    m_driveNetParams.networkBatchSize = DW_DRIVENET_BATCHSIZE_2;
    m_driveNetParams.networkPrecision = DW_DRIVENET_PRECISION_FP32;
    CHECK_DW_ERROR(dwDriveNet_initialize(&m_driveNet, &objectClusteringHandles, &driveNetClasses,
	                                 &numDriveNetClasses, m_sdk, &m_driveNetParams));

    // Initialize Objec Detector from DriveNet
    dwObjectDetectorDNNParams tmpDNNParams;
    CHECK_DW_ERROR(dwObjectDetector_initDefaultParams(&tmpDNNParams, &detectorParams));
    // Enable fusing objects from different ROIs
    detectorParams.enableFuseObjects = DW_TRUE;
    // Two images will be given as input. Each image is a region on the image received from camera.
    detectorParams.maxNumImages = 2U;
    CHECK_DW_ERROR(dwObjectDetector_initializeFromDriveNet(&m_driveNetDetector, m_sdk, m_driveNet,
	                                                   &detectorParams));
    CHECK_DW_ERROR(dwObjectDetector_setCUDAStream(m_cudaStream, m_driveNetDetector));

    // since our input images might have a different aspect ratio as the input to drivenet
    // we setup the ROI such that the crop happens from the top of the image
    float32_t aspectRatio = 1.0f;
    {
	dwBlobSize inputBlob;
	CHECK_DW_ERROR(dwDriveNet_getInputBlobsize(&inputBlob, m_driveNet));

	aspectRatio = static_cast<float32_t>(inputBlob.height) / static_cast<float32_t>(inputBlob.width);
    }

    // 1st image is a full resolution image as it comes out from the RawPipeline (cropped to DriveNet aspect ratio)
    dwRect fullROI;
    {
	fullROI = {0, 0, static_cast<int32_t>(m_cameraImageProperties.width),
	                 static_cast<int32_t>(m_cameraImageProperties.width * aspectRatio)};
	dwTransformation2D transformation = {{1.0f, 0.0f, 0.0f,
	                                      0.0f, 1.0f, 0.0f,
	                                      0.0f, 0.0f, 1.0f}};

	CHECK_DW_ERROR(dwObjectDetector_setROI(0, &fullROI, &transformation, m_driveNetDetector));
    }

    // 2nd image is a cropped out region within the 1/4-3/4 of the original image in the center
    {
	dwRect ROI = {fullROI.width/4, fullROI.height/4,
	              fullROI.width/2, fullROI.height/2};
	dwTransformation2D transformation = {{1.0f, 0.0f, 0.0f,
	                                      0.0f, 1.0f, 0.0f,
	                                      0.0f, 0.0f, 1.0f}};

	CHECK_DW_ERROR(dwObjectDetector_setROI(1, &ROI, &transformation, m_driveNetDetector));
    }

    // fill out member structure according to the ROIs
    CHECK_DW_ERROR(dwObjectDetector_getROI(&detectorParams.ROIs[0],
	           &detectorParams.transformations[0], 0, m_driveNetDetector));
    CHECK_DW_ERROR(dwObjectDetector_getROI(&detectorParams.ROIs[1],
	           &detectorParams.transformations[1], 1, m_driveNetDetector));

    // Get which label name for each class id
    classLabels.resize(numDriveNetClasses);
    for (uint32_t classIdx = 0U; classIdx < numDriveNetClasses; ++classIdx) {
	const char *classLabel;
	CHECK_DW_ERROR(dwDriveNet_getClassLabel(&classLabel, classIdx, m_driveNet));
	classLabels[classIdx] = classLabel;
    }

    // Initialize arrays for the pipeline
    objectProposals.resize(numDriveNetClasses, std::vector<dwObject>(maxProposalsPerClass));
    objectClusters.resize(numDriveNetClasses, std::vector<dwObject>(maxClustersPerClass));
    objectsTracked.resize(numDriveNetClasses, std::vector<dwObject>(maxClustersPerClass));
    objectsMerged.resize(numDriveNetClasses, std::vector<dwObject>(maxClustersPerClass));

    numTrackedObjects.resize(numDriveNetClasses, 0);
    numMergedObjects.resize(numDriveNetClasses, 0);
    numClusters.resize(numDriveNetClasses, 0);
    numProposals.resize(numDriveNetClasses, 0);

    dnnBoxList.resize(numDriveNetClasses);

    return true;
}

//#######################################################################################
bool PilotNet::initTracker()
{
    // initialize ObjectTracker - it will be required to track detected instances over multiple frames
    // for better understanding how ObjectTracker works see sample_object_tracker

    dwObjectFeatureTrackerParams featureTrackingParams;
    dwObjectTrackerParams objectTrackingParams[DW_OBJECT_MAX_CLASSES];
    CHECK_DW_ERROR(dwObjectTracker_initDefaultParams(&featureTrackingParams, objectTrackingParams,
                                                     numDriveNetClasses));
    featureTrackingParams.maxFeatureCount = 8000;
    featureTrackingParams.detectorScoreThreshold = 0.0001f;
    featureTrackingParams.iterationsLK = 10;
    featureTrackingParams.windowSizeLK = 8;

    for (uint32_t classIdx = 0U; classIdx < numDriveNetClasses; ++classIdx) {
        objectTrackingParams[classIdx].confRateTrackMax = 0.05f;
        objectTrackingParams[classIdx].confRateTrackMin = 0.01f;
        objectTrackingParams[classIdx].confRateDetect = 0.5f;
        objectTrackingParams[classIdx].confThreshDiscard = 0.0f;
        objectTrackingParams[classIdx].maxFeatureCountPerBox = 200;
    }

    {
        CHECK_DW_ERROR(dwObjectTracker_initialize(&m_objectTracker, m_sdk,
                                                  &m_cameraImageProperties, &featureTrackingParams,
                                                  objectTrackingParams, numDriveNetClasses));
    }

    CHECK_DW_ERROR(dwObjectTracker_setCUDAStream(m_cudaStream, m_objectTracker));

    return true;
}

//#######################################################################################
void PilotNet::drawLaneDetectionROI(dwRenderBufferHandle_t renderBuffer, dwRendererHandle_t renderer)
{
    dwRect roi{};
    dwLaneDetector_getDetectionROI(&roi, m_laneDetector);
    float32_t x_start = static_cast<float32_t>(roi.x);
    float32_t x_end   = static_cast<float32_t>(roi.x + roi.width);
    float32_t y_start = static_cast<float32_t>(roi.y);
    float32_t y_end   = static_cast<float32_t>(roi.y + roi.height);
    float32_t *coords     = nullptr;
    uint32_t maxVertices  = 0;
    uint32_t vertexStride = 0;
    dwRenderBuffer_map(&coords, &maxVertices, &vertexStride, renderBuffer);
    coords[0]  = x_start;
    coords[1]  = y_start;
    coords    += vertexStride;
    coords[0]  = x_start;
    coords[1]  = y_end;
    coords    += vertexStride;
    coords[0]  = x_start;
    coords[1]  = y_end;
    coords    += vertexStride;
    coords[0]  = x_end;
    coords[1]  = y_end;
    coords    += vertexStride;
    coords[0]  = x_end;
    coords[1]  = y_end;
    coords    += vertexStride;
    coords[0] = x_end;
    coords[1] = y_start;
    coords    += vertexStride;
    coords[0] = x_end;
    coords[1] = y_start;
    coords    += vertexStride;
    coords[0] = x_start;
    coords[1] = y_start;
    dwRenderBuffer_unmap(8, renderBuffer);
    dwRenderer_setColor(DW_RENDERER_COLOR_YELLOW, renderer);
    dwRenderer_setLineWidth(2, renderer);
    dwRenderer_renderBuffer(renderBuffer, renderer);
}

//#######################################################################################
void PilotNet::drawLaneMarkingsCustomColor(float32_t laneColors[][4], uint32_t nColors,
                                          const dwLaneDetection &lanes, float32_t laneWidth,
                                          dwRenderBufferHandle_t renderBuffer, dwRendererHandle_t renderer)
{
    drawLaneDetectionROI(renderBuffer, renderer);

    for (uint32_t i = 0; i < lanes.numLaneMarkings; ++i) {

        const dwLaneMarking& laneMarking = lanes.laneMarkings[i];

        dwLanePositionType category = laneMarking.positionType;

        if(category==DW_LANEMARK_POSITION_ADJACENT_LEFT)
            dwRenderer_setColor(laneColors[0 % nColors], renderer);
        else if(category==DW_LANEMARK_POSITION_EGO_LEFT)
            dwRenderer_setColor(laneColors[1 % nColors], renderer);
        else if(category==DW_LANEMARK_POSITION_EGO_RIGHT)
            dwRenderer_setColor(laneColors[2 % nColors], renderer);
        else if(category==DW_LANEMARK_POSITION_ADJACENT_RIGHT)
            dwRenderer_setColor(laneColors[3 % nColors], renderer);
        else
            dwRenderer_setColor(laneColors[4 % nColors], renderer);

        dwRenderer_setLineWidth(laneWidth, renderer);

        float32_t* coords = nullptr;
        uint32_t maxVertices = 0;
        uint32_t vertexStride = 0;
        dwRenderBuffer_map(&coords, &maxVertices, &vertexStride, renderBuffer);

        uint32_t n_verts = 0;
        dwVector2f previousP{};
        bool firstPoint = true;

        for (uint32_t j = 0; j < laneMarking.numPoints; ++j) {

            dwVector2f center;
            center.x = laneMarking.imagePoints[j].x;
            center.y = laneMarking.imagePoints[j].y;

            if (firstPoint) { // Special case for the first point
                previousP = center;
                firstPoint = false;
            }
            else {
                n_verts += 2;
                if(n_verts > maxVertices)
                    break;

                coords[0] = static_cast<float32_t>(previousP.x);
                coords[1] = static_cast<float32_t>(previousP.y);
                coords += vertexStride;

                coords[0] = static_cast<float32_t>(center.x);
                coords[1] = static_cast<float32_t>(center.y);
                coords += vertexStride;

                previousP = center;
            }
        }

        dwRenderBuffer_unmap(n_verts, renderBuffer);
        dwRenderer_renderBuffer(renderBuffer, renderer);
    }
}

//#######################################################################################
void PilotNet::drawLaneMarkings(const dwLaneDetection &lanes, float32_t laneWidth,
                               dwRenderBufferHandle_t renderBuffer, dwRendererHandle_t renderer)
{
    const float32_t DW_RENDERER_COLOR_DARKYELLOW[4]  = { 180.0f/255.0f, 180.0f/255.0f,  10.0f/255.0f, 1.0f};
    const float32_t DW_RENDERER_COLOR_CYAN[4]      = {   10.0f/255.0f,   230.0f/255.0f,   230.0f/255.0f, 1.0f};

    float32_t colors[5][4];

    memcpy(colors[0], DW_RENDERER_COLOR_CYAN, sizeof(colors[0]));
    memcpy(colors[1], DW_RENDERER_COLOR_RED, sizeof(colors[1]));
    memcpy(colors[2], DW_RENDERER_COLOR_GREEN, sizeof(colors[2]));
    memcpy(colors[3], DW_RENDERER_COLOR_BLUE, sizeof(colors[3]));
    memcpy(colors[4], DW_RENDERER_COLOR_DARKYELLOW, sizeof(colors[3]));

    drawLaneMarkingsCustomColor(colors, 5, lanes, laneWidth, renderBuffer, renderer);
}

//#######################################################################################
void PilotNet::drawFreeSpaceDetectionROI(dwRenderBufferHandle_t renderBuffer, dwRendererHandle_t renderer)
{
    dwRect roi{};
    dwFreeSpaceDetector_getDetectionROI(&roi, m_freeSpaceDetector);
    float32_t x_start = static_cast<float32_t>(roi.x);
    float32_t x_end   = static_cast<float32_t>(roi.x + roi.width);
    float32_t y_start = static_cast<float32_t>(roi.y);
    float32_t y_end   = static_cast<float32_t>(roi.y + roi.height);
    float32_t *coords     = nullptr;
    uint32_t maxVertices  = 0;
    uint32_t vertexStride = 0;
    dwRenderBuffer_map(&coords, &maxVertices, &vertexStride, renderBuffer);
    coords[0]  = x_start;
    coords[1]  = y_start;
    coords    += vertexStride;
    coords[0]  = x_start;
    coords[1]  = y_end;
    coords    += vertexStride;
    coords[0]  = x_start;
    coords[1]  = y_end;
    coords    += vertexStride;
    coords[0]  = x_end;
    coords[1]  = y_end;
    coords    += vertexStride;
    coords[0]  = x_end;
    coords[1]  = y_end;
    coords    += vertexStride;
    coords[0] = x_end;
    coords[1] = y_start;
    coords    += vertexStride;
    coords[0] = x_end;
    coords[1] = y_start;
    coords    += vertexStride;
    coords[0] = x_start;
    coords[1] = y_start;
    dwRenderBuffer_unmap(8, renderBuffer);
    dwRenderer_setColor(DW_RENDERER_COLOR_YELLOW, renderer);
    dwRenderer_setLineWidth(2, renderer);
    dwRenderer_renderBuffer(renderBuffer, renderer);
}

//#######################################################################################
void PilotNet::drawFreeSpaceBoundary(dwFreeSpaceDetection* boundary, dwRenderBufferHandle_t renderBuffer, dwRendererHandle_t renderer)
{
    drawFreeSpaceDetectionROI(renderBuffer, renderer);

    uint32_t n_verts = 0;
    float32_t* coords= nullptr;
    uint32_t maxVertices = 0;
    uint32_t vertexStride = 0;
    dwFreeSpaceBoundaryType category = boundary->boundaryType[0];
    float32_t maxWidth = 8.0; // 10 meters as a step, [0, 10) will have max line width
    float32_t witdhRatio = 0.8;
    float32_t dist2Width[20];
    dist2Width[0] = maxWidth;
    for(uint32_t i = 1; i < 20; i++)
        dist2Width[i] = dist2Width[i-1]*witdhRatio;

    float32_t prevWidth, curWidth = maxWidth/2;
    prevWidth = curWidth;

    dwRenderer_setLineWidth(prevWidth, renderer);

    if(category==DW_BOUNDARY_TYPE_OTHER)
        dwRenderer_setColor(DW_RENDERER_COLOR_YELLOW, renderer);
    else if(category==DW_BOUNDARY_TYPE_CURB)
        dwRenderer_setColor(DW_RENDERER_COLOR_GREEN, renderer);
    else if(category==DW_BOUNDARY_TYPE_VEHICLE)
        dwRenderer_setColor(DW_RENDERER_COLOR_RED, renderer);
    else if(category==DW_BOUNDARY_TYPE_PERSON)
        dwRenderer_setColor(DW_RENDERER_COLOR_BLUE, renderer);

    dwRenderBuffer_map(&coords, &maxVertices, &vertexStride, renderBuffer);

    for (uint32_t i = 1; i < boundary->numberOfBoundaryPoints; ++i) {
        if(boundary->boundaryType[i] != boundary->boundaryType[i-1] || curWidth != prevWidth) {
            dwRenderBuffer_unmap(n_verts, renderBuffer);
            dwRenderer_renderBuffer(renderBuffer, renderer);

            coords = nullptr;
            maxVertices = 0;
            vertexStride = 0;
            n_verts = 0;
            dwRenderer_setLineWidth(curWidth, renderer);

            category = boundary->boundaryType[i];
            if(category==DW_BOUNDARY_TYPE_OTHER)
                dwRenderer_setColor(DW_RENDERER_COLOR_YELLOW, renderer);
            else if(category==DW_BOUNDARY_TYPE_CURB)
                  dwRenderer_setColor(DW_RENDERER_COLOR_GREEN, renderer);
            else if(category==DW_BOUNDARY_TYPE_VEHICLE)
                  dwRenderer_setColor(DW_RENDERER_COLOR_RED, renderer);
            else if(category==DW_BOUNDARY_TYPE_PERSON)
                  dwRenderer_setColor(DW_RENDERER_COLOR_BLUE, renderer);

            dwRenderBuffer_map(&coords, &maxVertices, &vertexStride, renderBuffer);
        }
        n_verts += 2;
        if(n_verts > maxVertices)
            break;

        coords[0] = static_cast<float32_t>(boundary->boundaryImagePoint[i-1].x);
        coords[1] = static_cast<float32_t>(boundary->boundaryImagePoint[i-1].y);
        coords += vertexStride;

        coords[0] = static_cast<float32_t>(boundary->boundaryImagePoint[i].x);
        coords[1] = static_cast<float32_t>(boundary->boundaryImagePoint[i].y);
        coords += vertexStride;
        prevWidth = curWidth;
    }

    dwRenderBuffer_unmap(n_verts, renderBuffer);
    dwRenderer_renderBuffer(renderBuffer, renderer);
}

//#######################################################################################
void PilotNet::renderCameraTexture(dwImageStreamerHandle_t streamer, dwRendererHandle_t renderer)
{
    dwImageGL *frameGL = nullptr;

    // Render input image
    // setRendererRect(0, 0);
    // dwRenderer_renderTexture(frameGL->tex, frameGL->target, renderer);
    // dwRenderer_setColor(DW_RENDERER_COLOR_GREEN, m_renderer);
    // dwRenderer_renderText(20, 20, "Input image", m_renderer);

    if (dwImageStreamer_receiveGL(&frameGL, 30000, streamer) != DW_SUCCESS) {
        std::cerr << "did not received GL frame within 30ms" << std::endl;
    } else {
        // render received texture
        dwRenderer_renderTexture(frameGL->tex, frameGL->target, renderer);
        dwImageStreamer_returnReceivedGL(frameGL, streamer);
    }
}

//#######################################################################################
bool PilotNet::createVideoReplay(dwSensorHandle_t &salSensor,
                                      float32_t &cameraFrameRate,
                                      dwSALHandle_t sal)
{
    dwSensorParams params;
    dwStatus result;

    if (m_inputType.compare("camera") == 0) {
        std::string cameraType = gArguments.get("camera-type");
        std::string parameterString = "camera-type=" + cameraType;
        parameterString += ",csi-port=" + gArguments.get("csi-port");
        parameterString += ",slave=" + gArguments.get("slave");
        parameterString += ",serialize=false,camera-count=4";
        if(cameraType.compare("c-ov10640-b1") == 0 ||
                cameraType.compare("ov10640-svc210") == 0 ||
                cameraType.compare("ov10640-svc212") == 0)
        {
            parameterString += ",output-format=yuv";
            m_isRaw = false;
        }
        else{
            parameterString += ",output-format=raw";
            m_isRaw = true;
        }
        std::string cameraMask[4] = {"0001", "0010", "0100", "1000"};
        uint32_t cameraIdx = std::stoi(gArguments.get("camera-index"));
        if(cameraIdx < 0 || cameraIdx > 3){
            std::cerr << "Error: camera index must be 0, 1, 2 or 3" << std::endl;
            return false;
        }
        parameterString += ",camera-mask=" + cameraMask[cameraIdx];

        params.parameters           = parameterString.c_str();
        params.protocol             = "camera.gmsl";

        result                      = dwSAL_createSensor(&salSensor, params, sal);
        if (result != DW_SUCCESS) {
            std::cerr << "Cannot create driver: camera.gmsl with params: " << params.parameters << std::endl
                      << "Error: " << dwGetStatusName(result) << std::endl;
            return false;
        }
    }
    else{
        std::string parameterString = gArguments.parameterString();
        params.parameters           = parameterString.c_str();
        params.protocol             = "camera.virtual";
        result                      = dwSAL_createSensor(&salSensor, params, sal);
        if (result != DW_SUCCESS) {
            std::cerr << "Cannot create driver: camera.virtual with params: " << params.parameters << std::endl
                      << "Error: " << dwGetStatusName(result) << std::endl;
            return false;
        }

        std::size_t found = m_videoFilename.find_last_of(".");
        m_isRaw = m_videoFilename.substr(found+1).compare("raw") == 0 ? true : false;
    }

    dwSensorCamera_getSensorProperties(&m_cameraProperties, salSensor);
    cameraFrameRate = m_cameraProperties.framerate;

    if(m_isRaw)
        dwSensorCamera_getImageProperties(&m_cameraImageProperties, DW_CAMERA_RAW_IMAGE, salSensor);
    else
        dwSensorCamera_getImageProperties(&m_cameraImageProperties, DW_CAMERA_PROCESSED_IMAGE, salSensor);

    if(m_isRaw && m_inputType.compare("camera") == 0)
        m_cameraImageProperties.height = m_cameraProperties.resolution.y;
    m_cameraHeight = m_cameraImageProperties.height;
    m_cameraWidth = m_cameraImageProperties.width;

    return true;
}

//#######################################################################################
void PilotNet::setupRenderer(dwRendererHandle_t &renderer, const dwRect &screenRect, dwContextHandle_t dwSdk)
{
    CHECK_DW_ERROR( dwRenderer_initialize(&renderer, dwSdk) );

    float32_t boxColor[4] = {0.0f,1.0f,0.0f,1.0f};
    dwRenderer_setColor(boxColor, renderer);
    dwRenderer_setLineWidth(2.0f, renderer);
    dwRenderer_setRect(screenRect, renderer);
}

//#######################################################################################
void PilotNet::setupLineBuffer(dwRenderBufferHandle_t &lineBuffer, unsigned int maxLines, dwContextHandle_t dwSdk)
{
    dwRenderBufferVertexLayout layout;
    layout.posFormat   = DW_RENDER_FORMAT_R32G32_FLOAT;
    layout.posSemantic = DW_RENDER_SEMANTIC_POS_XY;
    layout.colFormat   = DW_RENDER_FORMAT_NULL;
    layout.colSemantic = DW_RENDER_SEMANTIC_COL_NULL;
    layout.texFormat   = DW_RENDER_FORMAT_NULL;
    layout.texSemantic = DW_RENDER_SEMANTIC_TEX_NULL;
    dwRenderBuffer_initialize(&lineBuffer, layout, DW_RENDER_PRIM_LINELIST, maxLines, dwSdk);
    dwRenderBuffer_set2DCoordNormalizationFactors((float32_t)m_cameraWidth,
                                                  (float32_t)m_cameraHeight, lineBuffer);
}


//#######################################################################################
const std::vector<std::pair<dwBox2D,std::string>>& PilotNet::getResult(uint32_t classIdx)
{
    return dnnBoxList[classIdx];
}

//#######################################################################################
void PilotNet::inferDetectorAsync(const dwImageCUDA* rcbImage)
{
    // we feed two images to the DriveNet module, the first one will have full ROI
    // the second one, is the same image, however with an ROI cropped in the center
    const dwImageCUDA* rcbImagePtr[2] = {rcbImage, rcbImage};
    CHECK_DW_ERROR(dwObjectDetector_inferDeviceAsync(rcbImagePtr, 2U, m_driveNetDetector));
}

//#######################################################################################
void PilotNet::inferTrackerAsync(const dwImageCUDA* rcbImage)
{
    // track feature points on the rcb image
    CHECK_DW_ERROR(dwObjectTracker_featureTrackDeviceAsync(rcbImage, m_objectTracker));
}

//#######################################################################################
void PilotNet::processResults()
{
    CHECK_DW_ERROR(dwObjectDetector_interpretHost(2U, m_driveNetDetector));

    // for each detection class, we do
    for (uint32_t classIdx = 0U; classIdx < classLabels.size(); ++classIdx) {

    	// track detection from last frame given new feature tracker responses
    	CHECK_DW_ERROR(dwObjectTracker_boxTrackHost(objectsTracked[classIdx].data(), &numTrackedObjects[classIdx],
    	                                            objectsMerged[classIdx].data(), numMergedObjects[classIdx],
    	                                            classIdx, m_objectTracker));

    	// extract new detections from DriveNet
    	CHECK_DW_ERROR(dwObjectDetector_getDetectedObjects(objectProposals[classIdx].data(),
    	                                                   &numProposals[classIdx],
    	                                                   0U, classIdx, m_driveNetDetector));

    	// cluster proposals
    	CHECK_DW_ERROR(dwObjectClustering_cluster(objectClusters[classIdx].data(),
    	                                          &numClusters[classIdx],
    	                                          objectProposals[classIdx].data(),
    	                                          numProposals[classIdx],
    	                                          objectClusteringHandles[classIdx]));

    	// the new response should be at new location as detected by DriveNet
    	// in addition we have previously tracked response from last time
    	// we hence now merge both detections to find the actual response for the current frame
    	const dwObject *toBeMerged[2] = {objectsTracked[classIdx].data(),
                                         objectClusters[classIdx].data()};
    	const size_t sizes[2] = {numTrackedObjects[classIdx], numClusters[classIdx]};
    	CHECK_DW_ERROR(dwObject_merge(objectsMerged[classIdx].data(), &numMergedObjects[classIdx],
        	                      maxClustersPerClass, toBeMerged, sizes, 2U, 0.1f, 0.1f, m_sdk));

	// extract now the actual bounding box of merged response in pixel coordinates to render on screen
	dnnBoxList[classIdx].resize(numMergedObjects[classIdx]);

    	for (uint32_t objIdx = 0U; objIdx < numMergedObjects[classIdx]; ++objIdx) {
        	const dwObject &obj = objectsMerged[classIdx][objIdx];
        	dwBox2D &box = dnnBoxList[classIdx][objIdx].first;
        	box.x = static_cast<int32_t>(std::round(obj.box.x));
        	box.y = static_cast<int32_t>(std::round(obj.box.y));
        	box.width = static_cast<int32_t>(std::round(obj.box.width));
        	box.height = static_cast<int32_t>(std::round(obj.box.height));

        	dnnBoxList[classIdx][objIdx].second = classLabels[classIdx];
        }
    }
}

//#######################################################################################
void PilotNet::drawROI(dwRect roi, const float32_t color[4], dwRenderBufferHandle_t renderBuffer, dwRendererHandle_t renderer)
{
    float32_t x_start = static_cast<float32_t>(roi.x) ;
    float32_t x_end   = static_cast<float32_t>(roi.x + roi.width);
    float32_t y_start = static_cast<float32_t>(roi.y);
    float32_t y_end   = static_cast<float32_t>(roi.y + roi.height);

    float32_t *coords     = nullptr;
    uint32_t maxVertices  = 0;
    uint32_t vertexStride = 0;
    dwRenderBuffer_map(&coords, &maxVertices, &vertexStride, renderBuffer);
    coords[0]  = x_start;
    coords[1]  = y_start;
    coords    += vertexStride;
    coords[0]  = x_start;
    coords[1]  = y_end;
    coords    += vertexStride;
    coords[0]  = x_start;
    coords[1]  = y_end;
    coords    += vertexStride;
    coords[0]  = x_end;
    coords[1]  = y_end;
    coords    += vertexStride;
    coords[0]  = x_end;
    coords[1]  = y_end;
    coords    += vertexStride;
    coords[0] = x_end;
    coords[1] = y_start;
    coords    += vertexStride;
    coords[0] = x_end;
    coords[1] = y_start;
    coords    += vertexStride;
    coords[0] = x_start;
    coords[1] = y_start;
    dwRenderBuffer_unmap(8, renderBuffer);
    dwRenderer_setColor(color, renderer);
    dwRenderer_setLineWidth(2, renderer);
    dwRenderer_renderBuffer(renderBuffer, renderer);
}

//#######################################################################################
dwStatus PilotNet::runSingleCameraPipeline()
{
    dwStatus status = DW_SUCCESS;
/*
    if (m_isRaw) {
        status = runSingleCameraPipelineRaw();
    } else {
        status = runSingleCameraPipelineH264();
    }

    if (status == DW_END_OF_STREAM) {
        std::cout << "Camera reached end of stream" << std::endl;
        dwSensor_reset(m_cameraSensor);
        gRun = false;
    }
    else if (status != DW_SUCCESS) {
        gRun = false;
    }
*/
    return status;
}

//#######################################################################################
dwStatus PilotNet::runSingleCameraPipelineRaw()
{
    dwStatus result                 = DW_FAILURE;
    dwCameraFrameHandle_t frame     = nullptr;
    dwImageCUDA* frameCUDARaw       = nullptr;
    dwImageCPU *frameCPURaw         = nullptr;
    dwImageCUDA* retimg             = nullptr;
    const dwImageDataLines* dataLines;
#ifdef DW_USE_NVMEDIA
    dwImageNvMedia *frameNvMediaRaw = nullptr;
#endif

    result = dwSensorCamera_readFrame(&frame, 0, 1000000, m_cameraSensor);
    if (result == DW_END_OF_STREAM)
        return result;
    if (result != DW_SUCCESS && result != DW_END_OF_STREAM) {
        std::cerr << "readFrameNvMedia: " << dwGetStatusName(result) << std::endl;
        return result;
    }

    if (m_inputType.compare("camera") == 0) {
#ifdef DW_USE_NVMEDIA
        result = dwSensorCamera_getImageNvMedia(&frameNvMediaRaw, DW_CAMERA_RAW_IMAGE, frame);
#endif
    }
    else{
        result = dwSensorCamera_getImageCPU(&frameCPURaw, DW_CAMERA_RAW_IMAGE, frame);
    }
    if (result != DW_SUCCESS) {
        std::cerr << "Cannot get raw image: " << dwGetStatusName(result) << std::endl;
        return result;
    }

    result = dwSensorCamera_getDataLines(&dataLines, frame);
    if (result != DW_SUCCESS) {
        std::cerr << "Cannot get data lines: " << dwGetStatusName(result) << std::endl;
        return result;
    }

    if (m_inputType.compare("camera") == 0) {
#ifdef DW_USE_NVMEDIA
        result = dwImageStreamer_postNvMedia(frameNvMediaRaw, m_streamerInput2CUDA);
#endif
    }
    else{
        result = dwImageStreamer_postCPU(frameCPURaw, m_streamerInput2CUDA);
    }
    if (result != DW_SUCCESS) {
        std::cerr << "Cannot post image: " << dwGetStatusName(result) << std::endl;
        return result;
    }

    result = dwImageStreamer_receiveCUDA(&frameCUDARaw, 10000, m_streamerInput2CUDA);

    // Raw -> RCB
    dwSoftISP_bindRawInput(frameCUDARaw, m_softISP);
    CHECK_DW_ERROR(dwSoftISP_processDeviceAsync(DW_SOFT_ISP_PROCESS_TYPE_DEMOSAIC | DW_SOFT_ISP_PROCESS_TYPE_TONEMAP,
                                                m_softISP));

    // frame -> GL (rgba) - for rendering
    {
        result = dwImageStreamer_postCUDA(&m_frameCUDArgba, m_streamerCamera2GL);
        if (result != DW_SUCCESS) {
            std::cerr << "cannot post RGBA image" << dwGetStatusName(result) << std::endl;
            return result;
        }

        renderCameraTexture(m_streamerCamera2GL, m_renderer);

        result = dwImageStreamer_waitPostedCUDA(&retimg, 60000, m_streamerCamera2GL);
        if (result != DW_SUCCESS) {
            std::cerr << "Cannot wait post RGBA image" << dwGetStatusName(result) << std::endl;
            return result;
        }
    }

    dwImageStreamer_returnReceivedCUDA(frameCUDARaw, m_streamerInput2CUDA);

    runDetector(&m_frameCUDArgba, m_laneDetector, m_freeSpaceDetector, m_driveNetDetector, m_objectTracker);

    if (m_inputType.compare("camera") == 0) {
#ifdef DW_USE_NVMEDIA
        dwImageStreamer_waitPostedNvMedia(&frameNvMediaRaw, 10000, m_streamerInput2CUDA);
#endif
    }
    else{
        dwImageStreamer_waitPostedCPU(&frameCPURaw, 10000, m_streamerInput2CUDA);
    }

    dwSensorCamera_returnFrame(&frame);

    return DW_SUCCESS;
}

//#######################################################################################
dwStatus PilotNet::runSingleCameraPipelineH264()
{
    dwStatus result             = DW_FAILURE;
    dwCameraFrameHandle_t frame     = nullptr;
#ifdef DW_USE_NVMEDIA
    dwImageNvMedia *frameNvMediaYuv = nullptr;
    dwImageCUDA *imgCUDA            = nullptr;
    dwImageNvMedia *retimg          = nullptr;
#else
    dwImageCUDA *frameCUDAyuv       = nullptr;
    dwImageCUDA *retimg             = nullptr;
#endif

    result = dwSensorCamera_readFrame(&frame, 0, 50000, m_cameraSensor);
    if (result == DW_END_OF_STREAM)
        return result;
    if (result != DW_SUCCESS) {
        std::cout << "readFrameCUDA: " << dwGetStatusName(result) << std::endl;
        return result;
    }

#ifdef DW_USE_NVMEDIA
    result = dwSensorCamera_getImageNvMedia(&frameNvMediaYuv, DW_CAMERA_PROCESSED_IMAGE, frame);
#else
    result = dwSensorCamera_getImageCUDA(&frameCUDAyuv, DW_CAMERA_PROCESSED_IMAGE, frame);
#endif
    if (result != DW_SUCCESS) {
        std::cout << "getImage: " << dwGetStatusName(result) << std::endl;
        return result;
    }

    // YUV->RGBA
#ifdef DW_USE_NVMEDIA
    result = dwImageFormatConverter_copyConvertNvMedia(&m_frameNVMrgba, frameNvMediaYuv, m_converterNvMYuv2rgba);
#else

    result = dwImageFormatConverter_copyConvertCUDA(&m_frameCUDArgba, frameCUDAyuv, m_converterInput2Rgba, 0);

    if(result == DW_SUCCESS) {
        if (m_dump || m_takeScreenshot) {
            // takeInputFrameScreenshot();
            std::cout << "Cann convert to RGBA: " << dwGetStatusName(result) << std::endl;
        }
    }
    else {
        std::cout << "Cannot convert to RGBA: " << dwGetStatusName(result) << std::endl;
        return result;
    }

    // rectification
    result = dwRectifier_warp(&m_rectifiedImage, &m_frameCUDArgba, m_rectifier);

    if(result == DW_SUCCESS) {
        if (m_dump || m_takeScreenshot) {
            takeRectifiedFrameScreenshot();
        }
    }
    else {
         std::cerr << "Cannot unwarp:" << dwGetStatusName(result) << std::endl;
         return result;
    }

#endif

    if (result != DW_SUCCESS) {
        std::cout << "Cannot convert to RGBA: " << dwGetStatusName(result) << std::endl;
        return result;
    }

    // we can return the frame already now, we are working with a copy from now on
    dwSensorCamera_returnFrame(&frame);

    // frame -> GL (rgba) - for rendering
    {
#ifdef DW_USE_NVMEDIA
        result = dwImageStreamer_postNvMedia(&m_frameNVMrgba, m_streamerCamera2GL);
#else
        // result = dwImageStreamer_postCUDA(&m_frameCUDArgba, m_streamerCamera2GL);
        result = dwImageStreamer_postCUDA(&m_rectifiedImage, m_streamerCamera2GL);
#endif
        if (result != DW_SUCCESS) {
            std::cerr << "cannot post RGBA image" << dwGetStatusName(result) << std::endl;
            return result;
        }

        renderCameraTexture(m_streamerCamera2GL, m_renderer);

#ifdef DW_USE_NVMEDIA
        result = dwImageStreamer_waitPostedNvMedia(&retimg, 60000, m_streamerCamera2GL);
#else
        result = dwImageStreamer_waitPostedCUDA(&retimg, 60000, m_streamerCamera2GL);
#endif
        if (result != DW_SUCCESS) {
            std::cerr << "Cannot wait post RGBA image" << dwGetStatusName(result) << std::endl;
            return result;
        }
    }
#ifdef DW_USE_NVMEDIA
    // (nvmedia) NVMEDIA -> CUDA (rgba) - for processing
    // since DNN expects pitch linear cuda memory we cannot just post gFrameNVMrgba through the streamer
    // cause the outcome of the streamer would have block layout, but we need pitch
    // hence we perform one more extra YUV2RGBA conversion using CUDA
    {
        result = dwImageStreamer_postNvMedia(frameNvMediaYuv, m_streamerNvMedia2CUDA);
        if (result != DW_SUCCESS) {
            std::cerr << "Cannot post NvMedia frame " << dwGetStatusName(result) << std::endl;
            return result;
        }

        result = dwImageStreamer_receiveCUDA(&imgCUDA, 60000, m_streamerNvMedia2CUDA);
        if (result != DW_SUCCESS || imgCUDA == 0) {
            std::cerr << "did not received CUDA frame within 60ms" << std::endl;
            return result;
        }

        // copy convert into RGBA
        result = dwImageFormatConverter_copyConvertCUDA(&m_frameCUDArgba, imgCUDA, m_converterInput2Rgba, 0);
        if (result != DW_SUCCESS) {
            std::cerr << "Cannot convert to RGBA" << std::endl;
            return result;
        }

    }
#endif

    // runDetector(&m_frameCUDArgba, m_laneDetector, m_freeSpaceDetector, m_driveNetDetector, m_objectTracker);
    runDetector(&m_rectifiedImage, m_laneDetector, m_freeSpaceDetector, m_driveNetDetector, m_objectTracker);

#ifdef DW_USE_NVMEDIA
    dwImageStreamer_returnReceivedCUDA(imgCUDA, m_streamerNvMedia2CUDA);
    dwImageStreamer_waitPostedNvMedia(&retimg, 60000, m_streamerNvMedia2CUDA);
#endif

    return DW_SUCCESS;
}

//#######################################################################################
void PilotNet::runDetector(dwImageCUDA* frame, dwLaneDetectorHandle_t laneDetector, dwFreeSpaceDetectorHandle_t freeSpaceDetector, dwObjectDetectorHandle_t driveNetDetector, dwObjectTrackerHandle_t objectTracker)
{  
    auto beginPilotNetTime = std::chrono::high_resolution_clock::now();

    if (driveNetDetector && objectTracker) // DriveNet
    {
        auto beginDriveNetTime = std::chrono::high_resolution_clock::now();
	dwObjectDetectorParams drivenetParams{};
        // Detect, track and render objects
        inferDetectorAsync(frame);
        // inferTrackerAsync(frame);
        processResults();

	// simpleRenderer->setScreenRect(dwRect{0, 0, m_windowWidth, m_windowHeight});
	// simpleRenderer->renderQuad(m_imgGl);

	// set the normalization coordinates for boxes rendering
	// simpleRenderer->setRenderBufferNormCoords(getImageProperties().width, getImageProperties().height, DW_RENDER_PRIM_LINELIST);

	// render boxes with labels for each detected class
	for (size_t classIdx = 0; classIdx < getNumClasses(); classIdx++) 
        {
	    // render bounding box
            dwRenderer_setColor(boxColors[classIdx % MAX_BOX_COLORS], m_renderer);
            drawBoxesWithLabels(getResult(classIdx), static_cast<float32_t>(m_cameraImageProperties.width), static_cast<float32_t>(m_cameraImageProperties.height), m_renderBuffer, m_renderer);
        }

        // draw ROI of the first image
        drawROI(drivenetParams.ROIs[0], DW_RENDERER_COLOR_LIGHTBLUE, m_renderBuffer, m_renderer);

        // draw ROI of the second image
        drawROI(drivenetParams.ROIs[1], DW_RENDERER_COLOR_YELLOW, m_renderBuffer, m_renderer);

        // stop to save frame or take screenshot (will cause a delay)
        if (m_dump || m_takeScreenshot)
            takeDriveNetScreenshot(); 

	auto endDriveNetTime = std::chrono::high_resolution_clock::now();
        std::chrono::milliseconds timeSinceUpdate = std::chrono::duration_cast<std::chrono::milliseconds>(endDriveNetTime - beginDriveNetTime);
        std::cout << "01. DriveNet Framework is (in ms): " <<  timeSinceUpdate.count() << std::endl;
    }

    if (laneDetector) // LaneNet
    {
        auto beginLaneNetTime = std::chrono::high_resolution_clock::now();
        dwLaneDetection lanes{};
        dwStatus res = dwLaneDetector_processDeviceAsync(frame, laneDetector);
        res = res == DW_SUCCESS ? dwLaneDetector_interpretHost(laneDetector) : res;

        if (res != DW_SUCCESS)
        {
            std::cerr << "runDetector failed with: " << dwGetStatusName(res) << std::endl;
        }

        dwLaneDetector_getLaneDetections(&lanes, laneDetector);
        drawLaneMarkings(lanes, 6.0f, m_renderBuffer, m_renderer); 

        // stop to save frame or take screenshot (will cause a delay)
        if (m_dump || m_takeScreenshot)
            takeLaneNetScreenshot(); 

        auto endLaneNetTime = std::chrono::high_resolution_clock::now();
        std::chrono::milliseconds timeSinceUpdate = std::chrono::duration_cast<std::chrono::milliseconds>(endLaneNetTime - beginLaneNetTime);
        std::cout << "02. LaneNet Framework is (in ms): " <<  timeSinceUpdate.count() << std::endl;
    }

    if (freeSpaceDetector) // OpenRoadNet
    {
        auto beginOpenRoadNetTime = std::chrono::high_resolution_clock::now();
        dwFreeSpaceDetection boundary{};
        dwStatus res = dwFreeSpaceDetector_processDeviceAsync(frame, freeSpaceDetector);
        res = res == DW_SUCCESS ? dwFreeSpaceDetector_interpretHost(freeSpaceDetector) : res;

        if (res != DW_SUCCESS)
        {
            std::cerr << "runDetector failed with: " << dwGetStatusName(res) << std::endl;
        }

        dwFreeSpaceDetector_getBoundaryDetection(&boundary, freeSpaceDetector);
        drawFreeSpaceBoundary(&boundary, m_renderBuffer, m_renderer);

        // stop to save frame or take screenshot (will cause a delay)
        if (m_dump || m_takeScreenshot)
            takeOpenRoadNetScreenshot(); 

        auto endOpenRoadNetTime = std::chrono::high_resolution_clock::now();
        std::chrono::milliseconds timeSinceUpdate = std::chrono::duration_cast<std::chrono::milliseconds>(endOpenRoadNetTime - beginOpenRoadNetTime);
        std::cout << "03. OpenRoadNet Framework is (in ms): " <<  timeSinceUpdate.count() << std::endl;
    } 

    // stop to save frame or take screenshot (will cause a delay)
    if (m_dump || m_takeScreenshot)
        takePilotNetScreenshot(); 

    auto endPilotNetTime = std::chrono::high_resolution_clock::now();
    std::chrono::milliseconds timeSinceUpdate = std::chrono::duration_cast<std::chrono::milliseconds>(endPilotNetTime - beginPilotNetTime);
    std::cout << "04. PilotNet Framework is (in ms): " <<  timeSinceUpdate.count() << std::endl;
}

//#######################################################################################
void PilotNet::takeInputFrameScreenshot()
{
	/*
    dwImageStreamer_postCUDA(&m_frameCUDArgba, m_streamerCUDA2CPU);
    dwImageCPU *imageCPU;
    dwImageStreamer_receiveCPU(&imageCPU, 60, m_streamerCUDA2CPU);

    char fname[128];
    sprintf(fname, "InputFrame_%04u.png", m_screenshotInputFrameCount++);

    lodepng_encode32_file(("/usr/local/driveworks/data/samples/pilotnet/InputFrame/" + std::string(fname)).c_str(), imageCPU->data[0],
                           imageCPU->prop.width, imageCPU->prop.height);

    std::cout << "Input Frame Saved to " << ("/usr/local/driveworks/data/samples/pilotnet/InputFrame/" + std::string(fname)).c_str() << "\n";
    m_takeScreenshot = false;

    dwImageStreamer_returnReceivedCPU(imageCPU, m_streamerCUDA2CPU);
    dwImageCUDA *processedCUDA;
    dwStatus status = dwImageStreamer_waitPostedCUDA(&processedCUDA, 30000, m_streamerCUDA2CPU);
    if (status != DW_SUCCESS || processedCUDA != &m_frameCUDArgba) {
        std::cerr << "Cannot wait post RGBA image" << dwGetStatusName(status) << std::endl;
    }
    */
}

//#######################################################################################
void PilotNet::takeRectifiedFrameScreenshot()
{
	/*
    dwImageStreamer_postCUDA(&m_rectifiedImage, m_streamerCUDA2CPU);
    dwImageCPU *imageCPU;
    dwImageStreamer_receiveCPU(&imageCPU, 60, m_streamerCUDA2CPU);

    char fname[128];
    sprintf(fname, "RectifiedFrame_%04u.png", m_screenshotRectifiedFrameCount++);

    lodepng_encode32_file(("/usr/local/driveworks/data/samples/pilotnet/RectifiedFrame/" + std::string(fname)).c_str(), imageCPU->data[0],
                           imageCPU->prop.width, imageCPU->prop.height);

    std::cout << "Input Frame Saved to " << ("/usr/local/driveworks/data/samples/pilotnet/RectifiedFrame/" + std::string(fname)).c_str() << "\n";
    m_takeScreenshot = false;

    dwImageStreamer_returnReceivedCPU(imageCPU, m_streamerCUDA2CPU);
    dwImageCUDA *processedCUDA;
    dwStatus status = dwImageStreamer_waitPostedCUDA(&processedCUDA, 30000, m_streamerCUDA2CPU);
    if (status != DW_SUCCESS || processedCUDA != &m_frameCUDArgba) {
        std::cerr << "Cannot wait post RGBA image" << dwGetStatusName(status) << std::endl;
    }
    */
}

//#######################################################################################
void PilotNet::takeDriveNetScreenshot()
{
/*
    dwImageStreamer_postGL(&m_rectifiedImage, m_streamerCUDA2GL);
    dwImageGL *imageGL;
    dwImageStreamer_receiveGL(&imageGL, 60, m_streamerCUDA2GL);

    char fname[128];
    sprintf(fname, "DriveNet_%04u.png", m_screenshotDriveNetFrameCount++);

    lodepng_encode32_file(("/usr/local/driveworks/data/samples/pilotnet/DriveNet/" + std::string(fname)).c_str(), imageGL->data[0],
                           imageGL->prop.width, imageGL->prop.height);

    std::cout << "DriveNet Frame Saved to " << ("/usr/local/driveworks/data/samples/pilotnet/DriveNet/" + std::string(fname)).c_str() << "\n";
    m_takeScreenshot = false;

    dwImageStreamer_returnReceivedGL(imageGL, m_streamerCUDA2GL);
    dwImageCUDA *processedCUDA;

    dwStatus status = dwImageStreamer_waitPostedCUDA(&processedCUDA, 30000, m_streamerCUDA2GL);

    if (status != DW_SUCCESS || processedCUDA != &m_rectifiedImage) {
        std::cerr << "Cannot wait post RGBA image" << dwGetStatusName(status) << std::endl;
    } */
}

//#######################################################################################
void PilotNet::takeLaneNetScreenshot()
{
	/*
    dwImageStreamer_postCUDA(&m_frameCUDArgba, m_streamerCUDA2CPU);
    dwImageCPU *imageCPU;
    dwImageStreamer_receiveCPU(&imageCPU, 60, m_streamerCUDA2CPU);

    char fname[128];
    sprintf(fname, "LaneNet_%04u.png", m_screenshotLaneNetFrameCount++);

    lodepng_encode32_file(("/usr/local/driveworks/data/samples/pilotnet/LaneNet/" + std::string(fname)).c_str(), imageCPU->data[0],
                           imageCPU->prop.width, imageCPU->prop.height);

    std::cout << "LaneNet Frame Saved to " << ("/usr/local/driveworks/data/samples/pilotnet/LaneNet/" + std::string(fname)).c_str() << "\n";
    m_takeScreenshot = false;

    dwImageStreamer_returnReceivedCPU(imageCPU, m_streamerCUDA2CPU);
    dwImageCUDA *processedCUDA;
    dwStatus status = dwImageStreamer_waitPostedCUDA(&processedCUDA, 30000, m_streamerCUDA2CPU);
    if (status != DW_SUCCESS || processedCUDA != &m_frameCUDArgba) {
        std::cerr << "Cannot wait post RGBA image" << dwGetStatusName(status) << std::endl;
    }
    */
}

//#######################################################################################
void PilotNet::takeOpenRoadNetScreenshot()
{
	/*
    dwImageStreamer_postCUDA(&m_frameCUDArgba, m_streamerCUDA2CPU);
    dwImageCPU *imageCPU;
    dwImageStreamer_receiveCPU(&imageCPU, 60, m_streamerCUDA2CPU);

    char fname[128];
    sprintf(fname, "OpenRoadNet_%04u.png", m_screenshotOpenRoadNetFrameCount++);

    lodepng_encode32_file(("/usr/local/driveworks/data/samples/pilotnet/OpenRoadNet/" + std::string(fname)).c_str(), imageCPU->data[0],
                           imageCPU->prop.width, imageCPU->prop.height);

    std::cout << "OpenRoadNet Frame Saved to " << ("/usr/local/driveworks/data/samples/pilotnet/OpenRoadNet/" + std::string(fname)).c_str() << "\n";
    m_takeScreenshot = false;

    dwImageStreamer_returnReceivedCPU(imageCPU, m_streamerCUDA2CPU);
    dwImageCUDA *processedCUDA;
    dwStatus status = dwImageStreamer_waitPostedCUDA(&processedCUDA, 30000, m_streamerCUDA2CPU);
    if (status != DW_SUCCESS || processedCUDA != &m_frameCUDArgba) {
        std::cerr << "Cannot wait post RGBA image" << dwGetStatusName(status) << std::endl;
    }
    */
}

//#######################################################################################
void PilotNet::takePilotNetScreenshot()
{
	/*
    dwImageStreamer_postCUDA(&m_frameCUDArgba, m_streamerCUDA2CPU);
    dwImageCPU *imageCPU;
    dwImageStreamer_receiveCPU(&imageCPU, 60, m_streamerCUDA2CPU);

    char fname[128];
    sprintf(fname, "PilotNet_%04u.png", m_screenshotPilotNetFrameCount++);

    lodepng_encode32_file(("/usr/local/driveworks/data/samples/pilotnet/PilotNet/" + std::string(fname)).c_str(), imageCPU->data[0],
                           imageCPU->prop.width, imageCPU->prop.height);

    std::cout << "PilotNet Frame Saved to " << ("/usr/local/driveworks/data/samples/pilotnet/PilotNet/" + std::string(fname)).c_str() << "\n";
    m_takeScreenshot = false;

    dwImageStreamer_returnReceivedCPU(imageCPU, m_streamerCUDA2CPU);
    dwImageCUDA *processedCUDA;
    dwStatus status = dwImageStreamer_waitPostedCUDA(&processedCUDA, 30000, m_streamerCUDA2CPU);
    if (status != DW_SUCCESS || processedCUDA != &m_frameCUDArgba) {
        std::cerr << "Cannot wait post RGBA image" << dwGetStatusName(status) << std::endl;
    }
    */
}

//#######################################################################################
void PilotNet::onProcessKey(int key)  
{
        std::cout << "onProcessKey " << "\n"; 
        // take screenshot
        if (key == GLFW_KEY_S) {
            m_takeScreenshot = true;
            std::cout << "take screenshot " << "\n";            
        }
}

//#######################################################################################
void PilotNet::releaseModules()
{
    dwStatus status = DW_FAILURE;

    if (m_converterInput2Rgba != DW_NULL_HANDLE)
        dwImageFormatConverter_release(&m_converterInput2Rgba);

    if (m_frameCUDArgba.dptr[0])
        cudaFree(m_frameCUDArgba.dptr[0]);

    if (m_streamerCamera2GL != DW_NULL_HANDLE)
        dwImageStreamer_release(&m_streamerCamera2GL);

    if (m_streamerCUDA2CPU != DW_NULL_HANDLE)
        dwImageStreamer_release(&m_streamerCUDA2CPU);

    if(m_isRaw){
        if (m_streamerInput2CUDA != DW_NULL_HANDLE)
            dwImageStreamer_release(&m_streamerInput2CUDA);
        if (m_softISP != DW_NULL_HANDLE)
            dwSoftISP_release(&m_softISP);
        if (m_frameCUDArcb.dptr[0])
            cudaFree(m_frameCUDArcb.dptr[0]);
    }

#ifdef DW_USE_NVMEDIA
    if (m_converterNvMYuv2rgba != DW_NULL_HANDLE)
        dwImageFormatConverter_release(&m_converterNvMYuv2rgba);

    if (m_streamerNvMedia2CUDA != DW_NULL_HANDLE)
        dwImageStreamer_release(&m_streamerNvMedia2CUDA);

    if (m_frameNVMrgba.img != nullptr)
        NvMediaImageDestroy(m_frameNVMrgba.img);
#endif

    // to rectify fisheye lens distortion
    releaseOutputCUDAImage(&m_rectifiedImage);

    // m_driveNetParams

    // Release drivenet
    // dwDriveNet_release(&m_driveNet);
    dwObjectDetector_release(&m_driveNetDetector);
    dwObjectTracker_release(&m_objectTracker);

    dwLaneDetector_release(&m_laneDetector);
    dwFreeSpaceDetector_release(&m_freeSpaceDetector);

    dwCalibratedCamera_release(&m_cameraModelIn);
    dwCalibratedCamera_release(&m_cameraModelOut);

    dwRigConfiguration_release(&m_rigConfig);
    dwCameraRig_release(&m_cameraRig); 

    status = dwRectifier_release(&m_rectifier);
    if (status != DW_SUCCESS) {
        logError("Cannot release rectifier: %s\n", dwGetStatusName(status));
    }

    dwRenderBuffer_release(&m_renderBuffer);
    dwRenderer_release(&m_renderer);

    dwSensor_stop(m_cameraSensor);
    dwSAL_releaseSensor(&m_cameraSensor);

    // release used objects in correct order
    dwSAL_release(&m_sal);

    // m_inputType

    dwRelease(&m_sdk);
    dwLogger_release();
}

//#######################################################################################
void PilotNet::releaseOutputCUDAImage(dwImageCUDA* output)
{
    cudaFree(output->dptr[0]);
}
