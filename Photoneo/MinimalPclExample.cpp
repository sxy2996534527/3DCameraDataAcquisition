/*
 * Photoneo's API Example - MinimalPclExample.cpp
 * Defines the entry point for the console application.
 * Demonstrates the basic functionality of PhoXi device. This Example shows
 * how to convert Frame to PCL format
 */

#include <iostream>
#include <string>
#include <vector>
#if defined(_WIN32)
#include <windows.h>
#elif defined(__linux__)
#include <unistd.h>
#endif

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/io/pcd_io.h"
#include "pcl/common/io.h"
#include <pcl/visualization/cloud_viewer.h>
// #include "pcl/visualization/pcl_visualizer.h"
#define PHOXI_PCL_SUPPORT

#include "PhoXi.h"

int user_data;

// Run software trigger example
void startSoftwareTriggerExample(pho::api::PPhoXi &PhoXiDevice);
// Print out list of device info to standard output
void printDeviceInfoList(
        const std::vector<pho::api::PhoXiDeviceInformation> &DeviceList);
// Print out device info to standard output
void printDeviceInfo(const pho::api::PhoXiDeviceInformation &DeviceInfo);
// Print out frame info to standard output
void printFrameInfo(const pho::api::PFrame &Frame);
// Print out frame data to standard output
void printFrameData(const pho::api::PFrame &Frame);

// void 
// viewerOneOff (pcl::visualization::PCLVisualizer& viewer)
// {
//     viewer.setBackgroundColor (0.0, 0.0, 0.0);
//     pcl::PointXYZ o;
//     o.x = 1.0;
//     o.y = 0;
//     o.z = 0;
//     viewer.addSphere (o, 0.25, "sphere", 0);
//     std::cout << "i only run once" << std::endl;
    
// }
    
void 
viewerPsycho (pcl::visualization::PCLVisualizer& viewer)
{
    static unsigned count = 0;
    std::stringstream ss;
    ss << "Once per viewer loop: " << count++;
    viewer.removeShape ("text", 0);
    viewer.addText (ss.str(), 200, 300, "text", 0);
    
    //FIXME: possible race condition here:
    user_data++;
}

int main(int argc, char *argv[]) {
    pho::api::PhoXiFactory Factory;

    // Check if the PhoXi Control Software is running
    if (!Factory.isPhoXiControlRunning()) {
        std::cout << "PhoXi Control Software is not running" << std::endl;
        return 0;
    }

    // Get List of available devices on the network
    std::vector<pho::api::PhoXiDeviceInformation> DeviceList =
            Factory.GetDeviceList();
    if (DeviceList.empty()) {
        std::cout << "PhoXi Factory has found 0 devices" << std::endl;
        return 0;
    }
    printDeviceInfoList(DeviceList);

    // Try to connect device opened in PhoXi Control, if any
    pho::api::PPhoXi PhoXiDevice = Factory.CreateAndConnectFirstAttached();
    if (PhoXiDevice) {
        std::cout << "You have already PhoXi device opened in PhoXi Control, "
                     "the API Example is connected to device: "
                  << (std::string)PhoXiDevice->HardwareIdentification
                  << std::endl;
    } else {
        std::cout
                << "You have no PhoXi device opened in PhoXi Control, the API ";
        for (size_t i = 0; i < DeviceList.size(); i++) {
            std::cout << "Example will try to connect to ..."
                      << DeviceList.at(i).HWIdentification << std::endl;
            // wait 5 second for scanner became ready
            PhoXiDevice = Factory.CreateAndConnect(
                    DeviceList.at(i).HWIdentification, 5000);
            if (PhoXiDevice) {
                std::cout << "succesfully connected" << std::endl;
                break;
            }
            if (i == DeviceList.size() - 1) {
                std::cout << "Can not connect to any device" << std::endl;
            }
        }
    }

    // Check if device was created
    if (!PhoXiDevice) {
        std::cout << "Your device was not created!" << std::endl;
        return 0;
    }

    // Check if device is connected
    if (!PhoXiDevice->isConnected()) {
        std::cout << "Your device is not connected" << std::endl;
        return 0;
    }

    // Launch software trigger example
    startSoftwareTriggerExample(PhoXiDevice);

    // Disconnect PhoXi device
    PhoXiDevice->Disconnect();
    return 0;
}

void convertToPCL(const pho::api::PFrame &Frame) {
    std::cout << "Frame " << Frame << "\n";
    pho::api::PhoXiTimeout timeout;
    pcl::PointCloud<pcl::PointXYZ>::Ptr PCLCloud(
            new pcl::PointCloud<pcl::PointXYZ>());

    pcl::PointCloud<pcl::PointXYZRGBNormal> MyPCLCloud;
    Frame->ConvertTo(MyPCLCloud);

    std::cout << "Number of points in PCL Cloud : " << MyPCLCloud.points.size()
              << std::endl;
}

void startSoftwareTriggerExample(pho::api::PPhoXi &PhoXiDevice) {
    if (PhoXiDevice->isAcquiring()) {
        // Stop acquisition to change trigger mode
        PhoXiDevice->StopAcquisition();
    }

    PhoXiDevice->TriggerMode = pho::api::PhoXiTriggerMode::Software;
    std::cout << "Software trigger mode was set" << std::endl;
    PhoXiDevice->ClearBuffer();
    PhoXiDevice->StartAcquisition();
    if (!PhoXiDevice->isAcquiring()) {
        std::cout << "Your device could not start acquisition!" << std::endl;
        return;
    }

    std::cout << "Triggering the 0-th frame" << std::endl;
    int FrameID = PhoXiDevice->TriggerFrame();
    if (FrameID < 0) {
        // If negative number is returned trigger was unsuccessful
        std::cout << "Trigger was unsuccessful!" << std::endl;
        return;
    } else {
        std::cout << "Frame was triggered, Frame Id: " << FrameID
                    << std::endl;
    }

    std::cout << "Waiting for frame " << std::endl;
    pho::api::PFrame Frame = PhoXiDevice->GetSpecificFrame(
            FrameID, pho::api::PhoXiTimeout::Infinity);

    if (Frame) {
        printFrameInfo(Frame);
        printFrameData(Frame);
        // convertToPCL(Frame);
        std::cout << "Frame " << Frame << "\n";
        pho::api::PhoXiTimeout timeout;
        pcl::PointCloud<pcl::PointXYZ>::Ptr PCLCloud(
                new pcl::PointCloud<pcl::PointXYZ>());
        //pcl::PointCloud<pcl::PointXYZ> PCLCloud;
        pcl::PointCloud<pcl::PointXYZRGBNormal> MyPCLCloud;
        Frame->ConvertTo(MyPCLCloud);
        pcl::copyPointCloud(MyPCLCloud,*PCLCloud);
        std::cout << "Number of points in PCL Cloud : " << PCLCloud->points.size() << std::endl;
        pcl::PCDWriter writer;
	    writer.write<pcl::PointXYZ>("frame_demo.pcd", *PCLCloud, false);

        //write data

        // Create 3D viewer and add point clouds
        pcl::visualization::PCLVisualizer viewer ("3D Viewer");
        viewer.setBackgroundColor (0, 0, 0);
        //int v1 (0);
        //int v2 (1);
        //viewer.createViewPort (0.0, 0.0, 0.5, 1.0, v1);
        //viewer.createViewPort (0.5, 0.0, 1.0, 1.0, v2);
        //pcl::visualization::CloudViewer viewer("Cloud Viewer");
        
        //viewer.showCloud(PCLCloud);

        //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> src_cloud_color_handler (source_cloud_ptr, 255, 255, 0);
        //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> tar_cloud_color_handler (target_cloud_filtered_ptr, 0, 255, 255);
        //viewer.addPointCloud (source_cloud_ptr, src_cloud_color_handler, "source cloud v1", v1);
        //viewer.addPointCloud (target_cloud_filtered_ptr, tar_cloud_color_handler, "target cloud v1", v1);
        //viewer.addPointCloud (target_cloud_filtered_ptr, tar_cloud_color_handler, "target cloud v2", v2);
        viewer.addPointCloud (PCLCloud);
        viewer.initCameraParameters ();
        //setViewerPose (viewer, scene_sensor_pose);

        // Add transformed point cloud to viewer
        //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> tf_cloud_color_handler (transformed_cloud_ptr, 0, 255, 0);
        //viewer.addPointCloud<pcl::PointXYZ> (transformed_cloud_ptr, tf_cloud_color_handler, "initial aligned cloud", v2);

        //--------------------
        // -----Main loop-----
        //--------------------
        while (!viewer.wasStopped ()) {
        viewer.spinOnce ();
        pcl_sleep(0.01);
        }
            
        // //This will get called once per visualization iteration
        // viewer.runOnVisualizationThread (viewerPsycho);
        // while (!viewer.wasStopped ())
        // {
        // //you can also do cool processing here
        // //FIXME: Note that this is running in a separate thread from viewerPsycho
        // //and you should guard against race conditions yourself...
        //     user_data++;
        //     //viewer.spinOnce();
        // }
    } else {
        std::cout << "Failed to retrieve the frame!" << std::endl;
    }
    PhoXiDevice->StopAcquisition();
}

void printDeviceInfoList(
        const std::vector<pho::api::PhoXiDeviceInformation> &DeviceList) {
    for (std::size_t i = 0; i < DeviceList.size(); ++i) {
        std::cout << "Device: " << i << std::endl;
        printDeviceInfo(DeviceList[i]);
    }
}

void printDeviceInfo(const pho::api::PhoXiDeviceInformation &DeviceInfo) {
    std::cout << "  Name:                    " << DeviceInfo.Name << std::endl;
    std::cout << "  Hardware Identification: " << DeviceInfo.HWIdentification << std::endl;
    std::cout << "  Type:                    " << std::string(DeviceInfo.Type) << std::endl;
    std::cout << "  Firmware version:        " << DeviceInfo.FirmwareVersion << std::endl;
    std::cout << "  Variant:                 " << DeviceInfo.Variant << std::endl;
    std::cout << "  IsFileCamera:            " << (DeviceInfo.IsFileCamera ? "Yes" : "No") << std::endl;
    std::cout << "  Status:                  "
              << (DeviceInfo.Status.Attached
                          ? "Attached to PhoXi Control. "
                          : "Not Attached to PhoXi Control. ")
              << (DeviceInfo.Status.Ready ? "Ready to connect" : "Occupied")
              << std::endl
              << std::endl;
}

void printFrameInfo(const pho::api::PFrame &Frame) {
    const pho::api::FrameInfo &FrameInfo = Frame->Info;
    std::cout << "  Frame params: " << std::endl;
    std::cout << "    Frame Index: " << FrameInfo.FrameIndex << std::endl;
    std::cout << "    Frame Timestamp: " << FrameInfo.FrameTimestamp << " s"
              << std::endl;
    std::cout << "    Frame Acquisition duration: " << FrameInfo.FrameDuration
              << " ms" << std::endl;
    std::cout << "    Frame Computation duration: "
              << FrameInfo.FrameComputationDuration << " ms" << std::endl;
    std::cout << "    Frame Transfer duration: "
              << FrameInfo.FrameTransferDuration << " ms" << std::endl;
    std::cout << "    Sensor Position: [" << FrameInfo.SensorPosition.x << "; "
              << FrameInfo.SensorPosition.y << "; "
              << FrameInfo.SensorPosition.z << "]" << std::endl;
    std::cout << "    Total scan count: " << FrameInfo.TotalScanCount << std::endl;
}

void printFrameData(const pho::api::PFrame &Frame) {
    if (Frame->Empty()) {
        std::cout << "Frame is empty.";
        return;
    }
    std::cout << "  Frame data: " << std::endl;
    if (!Frame->PointCloud.Empty()) {
        std::cout << "    PointCloud:    (" << Frame->PointCloud.Size.Width
                  << " x " << Frame->PointCloud.Size.Height
                  << ") Type: " << Frame->PointCloud.GetElementName()
                  << std::endl;
    }
    if (!Frame->NormalMap.Empty()) {
        std::cout << "    NormalMap:     (" << Frame->NormalMap.Size.Width
                  << " x " << Frame->NormalMap.Size.Height
                  << ") Type: " << Frame->NormalMap.GetElementName()
                  << std::endl;
    }
    if (!Frame->DepthMap.Empty()) {
        std::cout << "    DepthMap:      (" << Frame->DepthMap.Size.Width
                  << " x " << Frame->DepthMap.Size.Height
                  << ") Type: " << Frame->DepthMap.GetElementName()
                  << std::endl;
    }
    if (!Frame->ConfidenceMap.Empty()) {
        std::cout << "    ConfidenceMap: (" << Frame->ConfidenceMap.Size.Width
                  << " x " << Frame->ConfidenceMap.Size.Height
                  << ") Type: " << Frame->ConfidenceMap.GetElementName()
                  << std::endl;
    }
    if (!Frame->Texture.Empty()) {
        std::cout << "    Texture:       (" << Frame->Texture.Size.Width
                  << " x " << Frame->Texture.Size.Height
                  << ") Type: " << Frame->Texture.GetElementName() << std::endl;
    }
}
