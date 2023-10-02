// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#pragma once

#include <k4abttypes.h>
#include <BodyTrackingHelpers.h>

#include "WindowController3d.h"
#include <opencv2/opencv.hpp>

// This is a wrapper library that convert the types from the k4abt types to the window3d visualization library types
class Window3dWrapper
{
public:
    ~Window3dWrapper();

    // Create Window3d wrapper without point cloud shading
    void Create(
        const char* name,
        k4a_depth_mode_t depthMode,
        int windowWidth = -1,
        int windowHeight = -1);

    // Create Window3d wrapper with point cloud shading
    void Create(
        const char* name,
        const k4a_calibration_t& sensorCalibration,
        const k4a_calibration_t& sensorCalibration2,
        const k4a_calibration_t& sensorCalibration3,
        const k4a_calibration_t& sensorCalibration4,
        const k4a_calibration_t& sensorCalibration5,
        const k4a_calibration_t& sensorCalibration6,
        const k4a_calibration_t& sensorCalibration7,
        const k4a_calibration_t& sensorCalibration8
    );

    void SetCloseCallback(
        Visualization::CloseCallbackType closeCallback,
        void* closeCallbackContext = nullptr);

    void SetKeyCallback(
        Visualization::KeyCallbackType keyCallback,
        void* keyCallbackContext = nullptr);

    void Delete();

    void UpdatePointClouds(k4a_image_t depthImage, std::vector<Color> pointCloudColors = std::vector<Color>());

    void UpdatePointClouds_RGB(k4a_image_t depthImage, k4a_image_t colorImage, int num);

    void UpdatePointClouds_UV(k4a_image_t depthImage, k4a_image_t colorImage, int num, std::vector<float> extrinsic);
    
    void CleanJointsAndBones();

    void AddJoint(k4a_float3_t position, k4a_quaternion_t orientation, Color color, int num, std::vector<float> extrinsic);

    void AddBone(k4a_float3_t joint1Position, k4a_float3_t joint2Position, Color color, int num, std::vector<float> extrinsic);

    // Helper function to directly add the whole body for rendering instead of adding separate joints and bones
    void AddBody(const k4abt_body_t& body, Color color);

    void Render();
    void Render2();

    // Window Configuration Functions
    void SetFloorRendering(bool enableFloorRendering, float floorPositionX, float floorPositionY, float floorPositionZ);

    void SetWindowPosition(int xPos, int yPos);

    // Render Setting Functions
    void SetLayout3d(Visualization::Layout3d layout3d);
    void SetJointFrameVisualization(bool enableJointFrameVisualization);
    void ConcateBuffer();
    void ConcatePoint();

    cv::Mat create_distance_mat(cv::Mat depth_mat);
    cv::Mat create_depth_mat(cv::Mat dist_mat, cv::Mat depth_mat);


    struct jointcopy {
        int num;
        float x;
        float y;
        float z;
        float int_x;
        float int_y;
    };

    std::vector<jointcopy> m_joint_result_ref;
    std::vector<jointcopy> m_joint_result_src;

    std::vector<jointcopy> m_joint;
    std::vector<jointcopy> m_joint2;
    std::vector<jointcopy> m_joint3;
    std::vector<jointcopy> m_joint4;
    std::vector<jointcopy> m_joint5;
    std::vector<jointcopy> m_joint6;
    std::vector<jointcopy> m_joint7;
    std::vector<jointcopy> m_joint8;

    std::vector<jointcopy> m_joint_result;
    std::vector<jointcopy> m_joint_result2;
    std::vector<jointcopy> m_joint_result3;
    std::vector<jointcopy> m_joint_result4;
    std::vector<jointcopy> m_joint_result5;
    std::vector<jointcopy> m_joint_result6;
    std::vector<jointcopy> m_joint_result7;
    std::vector<jointcopy> m_joint_result8;

    std::vector<uint8_t> m_colorBuffer;

    std::vector<uint16_t> m_depthBuffer;
    std::vector<uint16_t> m_depthBuffer2;
    std::vector<uint16_t> m_depthBuffer3;
    std::vector<uint16_t> m_depthBuffer4;
    std::vector<uint16_t> m_depthBuffer5;
    std::vector<uint16_t> m_depthBuffer6;
    std::vector<uint16_t> m_depthBuffer7;
    std::vector<uint16_t> m_depthBuffer8;


    cv::Mat m_distBuffer;
    cv::Mat m_distBuffer2;
    cv::Mat m_distBuffer3;
    cv::Mat m_distBuffer4;
    cv::Mat m_distBuffer5;
    cv::Mat m_distBuffer6;
    cv::Mat m_distBuffer7;
    cv::Mat m_distBuffer8;

    //cv::Mat m_depthBuffer;
    //cv::Mat m_depthBuffer2;
    //cv::Mat m_depthBuffer3;
    //cv::Mat m_depthBuffer4;
    //cv::Mat m_depthBuffer5;
    //cv::Mat m_depthBuffer6;
    //cv::Mat m_depthBuffer7;
    //cv::Mat m_depthBuffer8;

    std::vector<Visualization::PointCloudVertex> m_pointClouds;
    std::vector<Visualization::PointCloudVertex> m_pointClouds2;
    std::vector<Visualization::PointCloudVertex> m_pointClouds3;
    std::vector<Visualization::PointCloudVertex> m_pointClouds4;
    std::vector<Visualization::PointCloudVertex> m_pointClouds5;
    std::vector<Visualization::PointCloudVertex> m_pointClouds6;
    std::vector<Visualization::PointCloudVertex> m_pointClouds7;
    std::vector<Visualization::PointCloudVertex> m_pointClouds8;


    std::vector<Visualization::PointCloudVertex> m_pointClouds_t1;
    std::vector<Visualization::PointCloudVertex> m_pointClouds_t2;
    std::vector<Visualization::PointCloudVertex> m_pointClouds_t3;
    std::vector<Visualization::PointCloudVertex> m_pointClouds_t4;
    std::vector<Visualization::PointCloudVertex> m_pointClouds_t5;

    std::vector<Visualization::PointCloudVertex> m_pointClouds2_t1;
    std::vector<Visualization::PointCloudVertex> m_pointClouds2_t2;
    std::vector<Visualization::PointCloudVertex> m_pointClouds2_t3;
    std::vector<Visualization::PointCloudVertex> m_pointClouds2_t4;
    std::vector<Visualization::PointCloudVertex> m_pointClouds2_t5;

    std::vector<Visualization::PointCloudVertex> m_pointClouds3_t1;
    std::vector<Visualization::PointCloudVertex> m_pointClouds3_t2;
    std::vector<Visualization::PointCloudVertex> m_pointClouds3_t3;
    std::vector<Visualization::PointCloudVertex> m_pointClouds3_t4;
    std::vector<Visualization::PointCloudVertex> m_pointClouds3_t5;

    std::vector<Visualization::PointCloudVertex> m_pointClouds4_t1;
    std::vector<Visualization::PointCloudVertex> m_pointClouds4_t2;
    std::vector<Visualization::PointCloudVertex> m_pointClouds4_t3;
    std::vector<Visualization::PointCloudVertex> m_pointClouds4_t4;
    std::vector<Visualization::PointCloudVertex> m_pointClouds4_t5;

    std::vector<Visualization::PointCloudVertex> m_pointClouds5_t1;
    std::vector<Visualization::PointCloudVertex> m_pointClouds5_t2;
    std::vector<Visualization::PointCloudVertex> m_pointClouds5_t3;
    std::vector<Visualization::PointCloudVertex> m_pointClouds5_t4;
    std::vector<Visualization::PointCloudVertex> m_pointClouds5_t5;

    std::vector<Visualization::PointCloudVertex> m_pointClouds6_t1;
    std::vector<Visualization::PointCloudVertex> m_pointClouds6_t2;
    std::vector<Visualization::PointCloudVertex> m_pointClouds6_t3;
    std::vector<Visualization::PointCloudVertex> m_pointClouds6_t4;
    std::vector<Visualization::PointCloudVertex> m_pointClouds6_t5;

    std::vector<Visualization::PointCloudVertex> m_pointClouds7_t1;
    std::vector<Visualization::PointCloudVertex> m_pointClouds7_t2;
    std::vector<Visualization::PointCloudVertex> m_pointClouds7_t3;
    std::vector<Visualization::PointCloudVertex> m_pointClouds7_t4;
    std::vector<Visualization::PointCloudVertex> m_pointClouds7_t5;

    std::vector<Visualization::PointCloudVertex> m_pointClouds8_t1;
    std::vector<Visualization::PointCloudVertex> m_pointClouds8_t2;
    std::vector<Visualization::PointCloudVertex> m_pointClouds8_t3;
    std::vector<Visualization::PointCloudVertex> m_pointClouds8_t4;
    std::vector<Visualization::PointCloudVertex> m_pointClouds8_t5;


private:
    void InitializeCalibration(const k4a_calibration_t& sensorCalibration, int num);

    void BlendBodyColor(linmath::vec4 color, Color bodyColor);


    void UpdateColorBuffer(k4a_image_t colorImage);
   
    void UpdateColorBuffer2(k4a_image_t colorImage);

    void UpdateColorBuffer3(k4a_image_t colorImage);

    void UpdateColorBuffer4(k4a_image_t colorImage);

    void UpdateColorBuffer5(k4a_image_t colorImage);

    void UpdateColorBuffer6(k4a_image_t colorImage);

    void UpdateColorBuffer7(k4a_image_t colorImage);

    void UpdateColorBuffer8(k4a_image_t colorImage);
    
    bool CreateXYDepthTable(const k4a_calibration_t& sensorCalibration);

private:
    Visualization::WindowController3d m_window3d;

    bool m_pointCloudUpdated = false;
    //std::vector<uint16_t> m_depthBuffer;

    std::vector<uint8_t> m_colorBuffer2;
    std::vector<uint8_t> m_colorBuffer3;
    std::vector<uint8_t> m_colorBuffer4;
    std::vector<uint8_t> m_colorBuffer5;
    std::vector<uint8_t> m_colorBuffer6;
    std::vector<uint8_t> m_colorBuffer7;
    std::vector<uint8_t> m_colorBuffer8;



    std::vector<uint32_t> m_faceIndices;

    struct XY
    {
        float x;
        float y;
    };
    std::vector<XY> m_xyDepthTable;
    uint32_t m_depthWidth = 640;
    uint32_t m_depthHeight = 576;

    k4a_transformation_t m_transformationHandle = nullptr;
    k4a_transformation_t m_transformationHandle2 = nullptr;
    k4a_transformation_t m_transformationHandle3 = nullptr;
    k4a_transformation_t m_transformationHandle4 = nullptr;
    k4a_transformation_t m_transformationHandle5 = nullptr;
    k4a_transformation_t m_transformationHandle6 = nullptr;
    k4a_transformation_t m_transformationHandle7 = nullptr;
    k4a_transformation_t m_transformationHandle8 = nullptr;

    k4a_image_t color_image = nullptr;

    k4a_image_t depth_image = nullptr;
    k4a_image_t depth_image2 = nullptr;
    k4a_image_t depth_image3 = nullptr;
    k4a_image_t depth_image4 = nullptr;
    k4a_image_t depth_image5 = nullptr;
    k4a_image_t depth_image6 = nullptr;
    k4a_image_t depth_image7 = nullptr;
    k4a_image_t depth_image8 = nullptr;

    k4a_image_t transformed_color_image = nullptr;
    k4a_image_t transformed_color_image2 = nullptr;
    k4a_image_t transformed_color_image3 = nullptr;
    k4a_image_t transformed_color_image4 = nullptr;
    k4a_image_t transformed_color_image5 = nullptr;
    k4a_image_t transformed_color_image6 = nullptr;
    k4a_image_t transformed_color_image7 = nullptr;
    k4a_image_t transformed_color_image8 = nullptr;

    //k4a_image_t transformed_depth_image = nullptr;
    //k4a_image_t transformed_depth_image2 = nullptr;
    //k4a_image_t transformed_depth_image3 = nullptr;
    //k4a_image_t transformed_depth_image4 = nullptr;
    //k4a_image_t transformed_depth_image5 = nullptr;
    //k4a_image_t transformed_depth_image6 = nullptr;
    //k4a_image_t transformed_depth_image7 = nullptr;
    //k4a_image_t transformed_depth_image8 = nullptr;


    k4a_image_t m_pointCloudImage = nullptr;
    k4a_image_t m_pointCloudImage2 = nullptr;
    k4a_image_t m_pointCloudImage3 = nullptr;
    k4a_image_t m_pointCloudImage4 = nullptr;
    k4a_image_t m_pointCloudImage5 = nullptr;
    k4a_image_t m_pointCloudImage6 = nullptr;
    k4a_image_t m_pointCloudImage7 = nullptr;
    k4a_image_t m_pointCloudImage8 = nullptr;
};