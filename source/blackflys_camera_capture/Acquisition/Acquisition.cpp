//=============================================================================
// Copyright (c) 2001-2019 FLIR Systems, Inc. All Rights Reserved.
//
// This software is the confidential and proprietary information of FLIR
// Integrated Imaging Solutions, Inc. ("Confidential Information"). You
// shall not disclose such Confidential Information and shall use it only in
// accordance with the terms of the license agreement you entered into
// with FLIR Integrated Imaging Solutions, Inc. (FLIR).
//
// FLIR MAKES NO REPRESENTATIONS OR WARRANTIES ABOUT THE SUITABILITY OF THE
// SOFTWARE, EITHER EXPRESSED OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
// PURPOSE, OR NON-INFRINGEMENT. FLIR SHALL NOT BE LIABLE FOR ANY DAMAGES
// SUFFERED BY LICENSEE AS A RESULT OF USING, MODIFYING OR DISTRIBUTING
// THIS SOFTWARE OR ITS DERIVATIVES.
//=============================================================================

/**
 *  @example Acquisition.cpp
 *
 *  @brief Acquisition.cpp shows how to acquire images. It relies on
 *  information provided in the Enumeration example. Also, check out the
 *  ExceptionHandling and NodeMapInfo examples if you haven't already.
 *  ExceptionHandling shows the handling of standard and Spinnaker exceptions
 *  while NodeMapInfo explores retrieving information from various node types.
 *
 *  This example touches on the preparation and cleanup of a camera just before
 *  and just after the acquisition of images. Image retrieval and conversion,
 *  grabbing image data, and saving images are all covered as well.
 *
 *  Once comfortable with Acquisition, we suggest checking out
 *  AcquisitionMultipleCamera, NodeMapCallback, or SaveToAvi.
 *  AcquisitionMultipleCamera demonstrates simultaneously acquiring images from
 *  a number of cameras, NodeMapCallback serves as a good introduction to
 *  programming with callbacks and events, and SaveToAvi exhibits video creation.
 */

#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"
#include <iostream>
#include <sstream>
#include <chrono>
#include <iomanip>
#include <string>
#include <vector>
#include "date/tz.h"

#include <experimental/filesystem>
namespace fs = std::experimental::filesystem; // experimental before c++17

#include <opencv2/opencv.hpp>
#include <opencv2/viz.hpp>

using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;
//using namespace std;
using std::cout;
using std::endl;

std::string get_timestamp() 
{
    // auto local_time = date::make_zoned(date::current_zone(), std::chrono::system_clock::now());
    auto tp = std::chrono::system_clock::now();
    return date::format("%F %T", tp);
}

#ifdef _DEBUG
// Disables heartbeat on GEV cameras so debugging does not incur timeout errors
int DisableHeartbeat(INodeMap& nodeMap, INodeMap& nodeMapTLDevice)
{
    cout << "Checking device type to see if we need to disable the camera's heartbeat..." << endl << endl;

    //
    // Write to boolean node controlling the camera's heartbeat
    //
    // *** NOTES ***
    // This applies only to GEV cameras and only applies when in DEBUG mode.
    // GEV cameras have a heartbeat built in, but when debugging applications the
    // camera may time out due to its heartbeat. Disabling the heartbeat prevents
    // this timeout from occurring, enabling us to continue with any necessary debugging.
    // This procedure does not affect other types of cameras and will prematurely exit
    // if it determines the device in question is not a GEV camera.
    //
    // *** LATER ***
    // Since we only disable the heartbeat on GEV cameras during debug mode, it is better
    // to power cycle the camera after debugging. A power cycle will reset the camera
    // to its default settings.
    //
    CEnumerationPtr ptrDeviceType = nodeMapTLDevice.GetNode("DeviceType");
    if (!IsAvailable(ptrDeviceType) || !IsReadable(ptrDeviceType))
    {
        cout << "Error with reading the device's type. Aborting..." << endl << endl;
        return -1;
    }
    else
    {
        if (ptrDeviceType->GetIntValue() == DeviceType_GEV)
        {
            cout << "Working with a GigE camera. Attempting to disable heartbeat before continuing..." << endl << endl;
            CBooleanPtr ptrDeviceHeartbeat = nodeMap.GetNode("GevGVCPHeartbeatDisable");
            if (!IsAvailable(ptrDeviceHeartbeat) || !IsWritable(ptrDeviceHeartbeat))
            {
                cout << "Unable to disable heartbeat on camera. Continuing with execution as this may be non-fatal..."
                    << endl
                    << endl;
            }
            else
            {
                ptrDeviceHeartbeat->SetValue(true);
                cout << "WARNING: Heartbeat on GigE camera disabled for the rest of Debug Mode." << endl;
                cout << "         Power cycle camera when done debugging to re-enable the heartbeat..." << endl << endl;
            }
        }
        else
        {
            cout << "Camera does not use GigE interface. Resuming normal execution..." << endl << endl;
        }
    }
    return 0;
}
#endif

// This function acquires and saves 10 images from a device.
int AcquireImages(CameraPtr pCam, INodeMap& nodeMap, INodeMap& nodeMapTLDevice, const std::vector<std::string>& dst_paths)
{
    int result = 0;

    cout << endl << endl << "*** IMAGE ACQUISITION ***" << endl << endl;

    try
    {
        //
        // Set acquisition mode to continuous
        //
        // *** NOTES ***
        // Because the example acquires and saves 10 images, setting acquisition
        // mode to continuous lets the example finish. If set to single frame
        // or multiframe (at a lower number of images), the example would just
        // hang. This would happen because the example has been written to
        // acquire 10 images while the camera would have been programmed to
        // retrieve less than that.
        //
        // Setting the value of an enumeration node is slightly more complicated
        // than other node types. Two nodes must be retrieved: first, the
        // enumeration node is retrieved from the nodemap; and second, the entry
        // node is retrieved from the enumeration node. The integer value of the
        // entry node is then set as the new value of the enumeration node.
        //
        // Notice that both the enumeration and the entry nodes are checked for
        // availability and readability/writability. Enumeration nodes are
        // generally readable and writable whereas their entry nodes are only
        // ever readable.
        //
        // Retrieve enumeration node from nodemap
        CEnumerationPtr ptrAcquisitionMode = nodeMap.GetNode("AcquisitionMode");
        if (!IsAvailable(ptrAcquisitionMode) || !IsWritable(ptrAcquisitionMode))
        {
            cout << "Unable to set acquisition mode to continuous (enum retrieval). Aborting..." << endl << endl;
            return -1;
        }

        // Retrieve entry node from enumeration node
        CEnumEntryPtr ptrAcquisitionModeContinuous = ptrAcquisitionMode->GetEntryByName("Continuous");
        if (!IsAvailable(ptrAcquisitionModeContinuous) || !IsReadable(ptrAcquisitionModeContinuous))
        {
            cout << "Unable to set acquisition mode to continuous (entry retrieval). Aborting..." << endl << endl;
            return -1;
        }

        // Retrieve integer value from entry node
        const int64_t acquisitionModeContinuous = ptrAcquisitionModeContinuous->GetValue();

        // Set integer value from entry node as new value of enumeration node
        ptrAcquisitionMode->SetIntValue(acquisitionModeContinuous);

        cout << "Acquisition mode set to continuous..." << endl;

#ifdef _DEBUG
        cout << endl << endl << "*** DEBUG ***" << endl << endl;

        // If using a GEV camera and debugging, should disable heartbeat first to prevent further issues
        if (DisableHeartbeat(nodeMap, nodeMapTLDevice) != 0)
        {
            return -1;
        }

        cout << endl << endl << "*** END OF DEBUG ***" << endl << endl;
#endif

        //
        // Begin acquiring images
        //
        // *** NOTES ***
        // What happens when the camera begins acquiring images depends on the
        // acquisition mode. Single frame captures only a single image, multi
        // frame captures a set number of images, and continuous captures a
        // continuous stream of images. Because the example calls for the
        // retrieval of 10 images, continuous mode has been set.
        //
        // *** LATER ***
        // Image acquisition must be ended when no more images are needed.
        //
        pCam->BeginAcquisition();

        cout << "Acquiring images..." << endl;

        //
        // Retrieve device serial number for filename
        //
        // *** NOTES ***
        // The device serial number is retrieved in order to keep cameras from
        // overwriting one another. Grabbing image IDs could also accomplish
        // this.
        //
        gcstring deviceSerialNumber("");
        CStringPtr ptrStringSerial = nodeMapTLDevice.GetNode("DeviceSerialNumber");
        if (IsAvailable(ptrStringSerial) && IsReadable(ptrStringSerial))
        {
            deviceSerialNumber = ptrStringSerial->GetValue();

            cout << "Device serial number retrieved as " << deviceSerialNumber << "..." << endl;
        }
        cout << endl;
        
        // Create destination directories
        fs::path targetParent_image = dst_paths[0];
        fs::create_directories(targetParent_image); // Recursively create target directory if not existing.
        std::ofstream export_file(dst_paths[1], std::ios::out);

        // Retrieve, convert, and save images
        unsigned int imageCnt = 0;
        uint64_t last = 0;
        while (1)
        {
            try
            {
                //
                // Retrieve next received image
                //
                // *** NOTES ***
                // Capturing an image houses images on the camera buffer. Trying
                // to capture an image that does not exist will hang the camera.
                //
                // *** LATER ***
                // Once an image from the buffer is saved and/or no longer
                // needed, the image must be released in order to keep the
                // buffer from filling up.
                //
                ImagePtr pResultImage = pCam->GetNextImage();
                int width = pResultImage->GetWidth();
                int height = pResultImage->GetHeight();

                // Write timestamp
                // auto system_timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
                auto image_timestamp = pResultImage->GetTimeStamp();
                auto timestamp = get_timestamp();

                //
                // Ensure image completion
                //
                // *** NOTES ***
                // Images can easily be checked for completion. This should be
                // done whenever a complete image is expected or required.
                // Further, check image status for a little more insight into
                // why an image is incomplete.
                //
                if (pResultImage->IsIncomplete())
                {
                    // Retrieve and print the image status description
                    cout << "Image incomplete: " << Image::GetImageStatusDescription(pResultImage->GetImageStatus())
                        << "..." << endl
                        << endl;
                }
                else
                {
                    // *** NOTES ***
                    // Images have quite a bit of available metadata including
                    // things such as CRC, image status, and offset values, to
                    // name a few.
                    //
                    const size_t width = pResultImage->GetWidth();
                    const size_t height = pResultImage->GetHeight();

                    //
                    // Convert image to mono 8
                    //
                    // *** NOTES ***
                    // Images can be converted between pixel formats by using
                    // the appropriate enumeration value. Unlike the original
                    // image, the converted one does not need to be released as
                    // it does not affect the camera buffer.
                    //
                    // When converting images, color processing algorithm is an
                    // optional parameter.
                    //
                    // ImagePtr convertedImage = pResultImage->Convert(PixelFormat_Mono8, HQ_LINEAR);
                    ImagePtr converted_pResultImage = pResultImage->Convert(PixelFormat_BGR8);
                    cv::Mat img(height, width, CV_8UC3, converted_pResultImage->GetData());
                    
                    cv::resize(img, img, cv::Size(width/2, height/2));
                    // cv::imshow("Image", img);
                    // cvWaitKey(1);

                    // Create a unique filename
                    std::stringstream filename;
                    filename << std::setfill('0') << std::setw(10) << imageCnt << ".png";
                                    
                    // Save image
                    cv::imwrite( dst_paths[0] + filename.str(), img); 
                
                    // write timestamp
                    // cout << timestamp << endl;
                    export_file << timestamp << std::endl;
                    
                    imageCnt++;
                }
                auto now = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
                std::cout << 1.0 / ((now - last) / 1000000.0) << " fps" << std::endl;
                last = now;
                //
                // Release image
                //
                // *** NOTES ***
                // Images retrieved directly from the camera (i.e. non-converted
                // images) need to be released in order to keep from filling the
                // buffer.
                //
                pResultImage->Release();
            }
            catch (Spinnaker::Exception& e)
            {
                cout << "Error: " << e.what() << endl;
                export_file.close();  
                result = -1;
            }
        }

        //
        // End acquisition
        //
        // *** NOTES ***
        // Ending acquisition appropriately helps ensure that devices clean up
        // properly and do not need to be power-cycled to maintain integrity.
        //

        pCam->EndAcquisition();
    }
    catch (Spinnaker::Exception& e)
    {
        cout << "Error: " << e.what() << endl;
        return -1;
    }

    return result;
}

// This function prints the device information of the camera from the transport
// layer; please see NodeMapInfo example for more in-depth comments on printing
// device information from the nodemap.
int PrintDeviceInfo(INodeMap& nodeMap)
{
    cout << endl << "*** DEVICE INFORMATION ***" << endl << endl;

    try
    {
        FeatureList_t features;
        const CCategoryPtr category = nodeMap.GetNode("DeviceInformation");
        if (IsAvailable(category) && IsReadable(category))
        {
            category->GetFeatures(features);

            for (auto it = features.begin(); it != features.end(); ++it)
            {
                const CNodePtr pfeatureNode = *it;
                cout << pfeatureNode->GetName() << " : ";
                CValuePtr pValue = static_cast<CValuePtr>(pfeatureNode);
                cout << (IsReadable(pValue) ? pValue->ToString() : "Node not readable");
                cout << endl;
            }
        }
        else
        {
            cout << "Device control information not available." << endl;
        }
    }
    catch (Spinnaker::Exception& e)
    {
        cout << "Error: " << e.what() << endl;
        return -1;
    }

    return 0;
}

// This function acts as the body of the example; please see NodeMapInfo example
// for more in-depth comments on setting up cameras.
int RunSingleCamera(CameraPtr pCam, const std::vector<std::string>& dst_paths)
{
    int result;

    try
    {
        // Retrieve TL device nodemap and print device information
        INodeMap& nodeMapTLDevice = pCam->GetTLDeviceNodeMap();

        result = PrintDeviceInfo(nodeMapTLDevice);

        // Initialize camera
        pCam->Init();

        // Retrieve GenICam nodemap
        INodeMap& nodeMap = pCam->GetNodeMap();

        // Acquire images
        result = result | AcquireImages(pCam, nodeMap, nodeMapTLDevice, dst_paths);

        // Deinitialize camera
        pCam->DeInit();
    }
    catch (Spinnaker::Exception& e)
    {
        cout << "Error: " << e.what() << endl;
        result = -1;
    }

    return result;
}

// Example entry point; please see Enumeration example for more in-depth
// comments on preparing and cleaning up the system.
int main(int argc, char** argv)
{

    std::vector<std::string> dst_paths(2);
    if (argc == 2) {
        std::string dst = "../../../data/data_raw/";
        dst += argv[1];
        dst_paths[0] = dst + "/image_01/data/";
        dst_paths[1] = dst + "/image_01/timestamps.txt";
    }
    else 
    {
        cout << "Should come with one argument: folder_name\n";
        return 0;
    }

    // Since this application saves images in the current folder
    // we must ensure that we have permission to write to this folder.
    // If we do not have permission, fail right away.
    FILE* tempFile = fopen("test.txt", "w+");
    if (tempFile == nullptr)
    {
        cout << "Failed to create file in current folder.  Please check "
            "permissions."
            << endl;
        cout << "Press Enter to exit..." << endl;
        getchar();
        return -1;
    }
    fclose(tempFile);
    remove("test.txt");

    // Print application build information
    cout << "Application build date: " << __DATE__ << " " << __TIME__ << endl << endl;

    // Retrieve singleton reference to system object
    SystemPtr system = System::GetInstance();

    // Print out current library version
    const LibraryVersion spinnakerLibraryVersion = system->GetLibraryVersion();
    cout << "Spinnaker library version: " << spinnakerLibraryVersion.major << "." << spinnakerLibraryVersion.minor
        << "." << spinnakerLibraryVersion.type << "." << spinnakerLibraryVersion.build << endl
        << endl;

    // Retrieve list of cameras from the system
    CameraList camList = system->GetCameras();

    const unsigned int numCameras = camList.GetSize();

    cout << "Number of cameras detected: " << numCameras << endl << endl;

    // Finish if there are no cameras
    if (numCameras == 0)
    {
        // Clear camera list before releasing system
        camList.Clear();

        // Release system
        system->ReleaseInstance();

        cout << "Not enough cameras!" << endl;
        cout << "Done! Press Enter to exit..." << endl;
        getchar();

        return -1;
    }

    //
    // Create shared pointer to camera
    //
    // *** NOTES ***
    // The CameraPtr object is a shared pointer, and will generally clean itself
    // up upon exiting its scope. However, if a shared pointer is created in the
    // same scope that a system object is explicitly released (i.e. this scope),
    // the reference to the shared point must be broken manually.
    //
    // *** LATER ***
    // Shared pointers can be terminated manually by assigning them to nullptr.
    // This keeps releasing the system from throwing an exception.
    //
    CameraPtr pCam = nullptr;

    int result = 0;

    // Run example on each camera
    for (unsigned int i = 0; i < numCameras; i++)
    {
        // Select camera
        pCam = camList.GetByIndex(i);

        cout << endl << "Recording from camera " << i << "..." << endl;

        // Run example
        result = result | RunSingleCamera(pCam, dst_paths);
        cout << "Camera " << i << " example complete..." << endl << endl;
    }

    //
    // Release reference to the camera
    //
    // *** NOTES ***
    // Had the CameraPtr object been created within the for-loop, it would not
    // be necessary to manually break the reference because the shared pointer
    // would have automatically cleaned itself up upon exiting the loop.
    //
    pCam = nullptr;

    // Clear camera list before releasing system
    camList.Clear();

    // Release system
    system->ReleaseInstance();

    return result;
}
