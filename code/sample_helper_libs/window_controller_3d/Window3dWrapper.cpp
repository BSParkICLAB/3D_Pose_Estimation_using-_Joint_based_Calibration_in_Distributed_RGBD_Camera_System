// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include "Window3dWrapper.h"
#include <omp.h>
#include <array>
#include <k4a/k4a.h>
#include <k4abt.h>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include "Utilities.h"
#include <opencv2/opencv.hpp>

const float MillimeterToMeter = 0.001f;

void ConvertMillimeterToMeter(k4a_float3_t positionInMM, linmath::vec3 outPositionInMeter)
{
    outPositionInMeter[0] = positionInMM.v[0] * MillimeterToMeter;
    outPositionInMeter[1] = positionInMM.v[1] * MillimeterToMeter;
    outPositionInMeter[2] = positionInMM.v[2] * MillimeterToMeter;
}

Window3dWrapper::~Window3dWrapper()
{
    Delete();
}

void Window3dWrapper::Create(
    const char* name,
    k4a_depth_mode_t depthMode,
    int windowWidth,
    int windowHeight)
{
    m_window3d.Create(name, true, windowWidth, windowHeight);
    m_window3d.SetMirrorMode(false);

    switch (depthMode)
    {
    case K4A_DEPTH_MODE_WFOV_UNBINNED:
    case K4A_DEPTH_MODE_WFOV_2X2BINNED:
        m_window3d.SetDefaultVerticalFOV(120.0f);
        break;
    case K4A_DEPTH_MODE_NFOV_2X2BINNED:
    case K4A_DEPTH_MODE_NFOV_UNBINNED:
    default:
        m_window3d.SetDefaultVerticalFOV(65.0f);
        break;
    }
}

void Window3dWrapper::Create(
    const char* name,
    const k4a_calibration_t& sensorCalibration,
    const k4a_calibration_t& sensorCalibration2,
    const k4a_calibration_t& sensorCalibration3,
    const k4a_calibration_t& sensorCalibration4,
    const k4a_calibration_t& sensorCalibration5,
    const k4a_calibration_t& sensorCalibration6,
    const k4a_calibration_t& sensorCalibration7,
    const k4a_calibration_t& sensorCalibration8
)
{
    Create(name, sensorCalibration.depth_mode);
    InitializeCalibration(sensorCalibration, 0);
    InitializeCalibration(sensorCalibration2, 1);
    InitializeCalibration(sensorCalibration3, 2);
    InitializeCalibration(sensorCalibration4, 3);    
    InitializeCalibration(sensorCalibration5, 4);
    InitializeCalibration(sensorCalibration6, 5);
    InitializeCalibration(sensorCalibration7, 6);
    InitializeCalibration(sensorCalibration8, 7);
    
}

void Window3dWrapper::SetCloseCallback(
    Visualization::CloseCallbackType closeCallback,
    void* closeCallbackContext)
{
    m_window3d.SetCloseCallback(closeCallback, closeCallbackContext);
}

void Window3dWrapper::SetKeyCallback(
    Visualization::KeyCallbackType keyCallback,
    void* keyCallbackContext)
{
    m_window3d.SetKeyCallback(keyCallback, keyCallbackContext);
}

void Window3dWrapper::Delete()
{
    m_window3d.Delete();

    if (m_transformationHandle != nullptr)
    {
        k4a_transformation_destroy(m_transformationHandle);
        m_transformationHandle = nullptr;
    }

    if (m_pointCloudImage != nullptr)
    {
        k4a_image_release(m_pointCloudImage);
        m_pointCloudImage = nullptr;
    }
}

void Window3dWrapper::UpdatePointClouds(k4a_image_t depthImage, std::vector<Color> pointCloudColors)
{
    m_pointCloudUpdated = true;
    VERIFY(k4a_transformation_depth_image_to_point_cloud(m_transformationHandle,
        depthImage,
        K4A_CALIBRATION_TYPE_DEPTH,
        m_pointCloudImage), "Transform depth image to point clouds failed!");

    int width = k4a_image_get_width_pixels(m_pointCloudImage);
    int height = k4a_image_get_height_pixels(m_pointCloudImage);

    int16_t* pointCloudImageBuffer = (int16_t*)k4a_image_get_buffer(m_pointCloudImage);

    for (int h = 0; h < height; h++)
    {
        for (int w = 0; w < width; w++)
        {
            int pixelIndex = h * width + w;
            k4a_float3_t position = {
                static_cast<float>(pointCloudImageBuffer[3 * pixelIndex + 0]),
                static_cast<float>(pointCloudImageBuffer[3 * pixelIndex + 1]),
                static_cast<float>(pointCloudImageBuffer[3 * pixelIndex + 2]) };

            // When the point cloud is invalid, the z-depth value is 0.
            if (position.v[2] == 0)
            {
                continue;
            }

            linmath::vec4 color = { 0.8f, 0.8f, 0.8f, 1.f };
            linmath::ivec2 pixelLocation = { w, h };

            if (pointCloudColors.size() > 0)
            {
                BlendBodyColor(color, pointCloudColors[pixelIndex]);
            }

            linmath::vec3 positionInMeter;
            ConvertMillimeterToMeter(position, positionInMeter);
            Visualization::PointCloudVertex pointCloud;
            linmath::vec3_copy(pointCloud.Position, positionInMeter);
            //linmath::vec4_copy(pointCloud.Color, color);
            //pointCloud.PixelLocation[0] = pixelLocation[0];
            //pointCloud.PixelLocation[1] = pixelLocation[1];

            m_pointClouds.push_back(pointCloud);
        }
    }

    //UpdateDepthBuffer(depthImage);
}


void Window3dWrapper::UpdatePointClouds_RGB(k4a_image_t depthImage, k4a_image_t colorImage, int num)
{
    m_pointCloudUpdated = true;
    k4a_transformation_t transformation_handle_value;
    if(num==0){
        transformation_handle_value = m_transformationHandle;
    }
    else {
        transformation_handle_value = m_transformationHandle2;
    }
    VERIFY(k4a_transformation_color_image_to_depth_camera(transformation_handle_value,
        depthImage,
        colorImage,
        transformed_color_image), "Transform color image to depth camera failed!");

    VERIFY(k4a_transformation_depth_image_to_point_cloud(transformation_handle_value,
        depthImage,
        K4A_CALIBRATION_TYPE_DEPTH,
        m_pointCloudImage), "Transform depth image to point clouds failed!");


    int width = k4a_image_get_width_pixels(m_pointCloudImage); 
    int height = k4a_image_get_height_pixels(m_pointCloudImage);
    int16_t* pointCloudImageBuffer = (int16_t*)k4a_image_get_buffer(m_pointCloudImage);
    const uint8_t* color_image_data = (uint8_t*)k4a_image_get_buffer(transformed_color_image);

#pragma omp parallel 
    {
    #pragma omp for schedule(static)
        for (int h = 0; h < height; h++)
        {
            for (int w = 0; w < width; w++)
            {
                int pixelIndex = h * width + w;
  
                if(static_cast<float>(pointCloudImageBuffer[3 * pixelIndex + 0]) / 1000 > -0.6f  && static_cast<float>(pointCloudImageBuffer[3 * pixelIndex + 1]) / 1000 > -1.0f &&static_cast<float>(pointCloudImageBuffer[3 * pixelIndex + 2])/1000 < 1.8f){
                k4a_float3_t position = {
                    static_cast<float>(pointCloudImageBuffer[3 * pixelIndex + 0]),
                    static_cast<float>(pointCloudImageBuffer[3 * pixelIndex + 1]),
                    static_cast<float>(pointCloudImageBuffer[3 * pixelIndex + 2]) };
                linmath::vec4 color = { color_image_data[4 * pixelIndex + 2] / 256.0f, color_image_data[4 * pixelIndex + 1] / 256.0f, color_image_data[4 * pixelIndex + 0] / 256.0f, 1.0f };
                linmath::ivec2 pixelLocation = { w, h };

                // When the point cloud is invalid, the z-depth value is 0.
                if (position.v[2] == 0)
                {
                    continue;
                }

                linmath::vec3 positionInMeter;
                ConvertMillimeterToMeter(position, positionInMeter);
                Visualization::PointCloudVertex pointCloud;
                linmath::vec3_copy(pointCloud.Position, positionInMeter);
                //linmath::vec4_copy(pointCloud.Color, color);
                //pointCloud.PixelLocation[0] = pixelLocation[0];
                //pointCloud.PixelLocation[1] = pixelLocation[1];

                m_pointClouds.push_back(pointCloud);
                }
            }
        }

        //UpdateDepthBuffer(depthImage);
    }
}



cv::Mat Window3dWrapper::create_distance_mat(cv::Mat depth_mat) {
    //distance map
    cv::Mat dist_mat;
    cv::Mat binary_depth = cv::Mat::zeros(cv::Size(640, 576), CV_8UC1);

    //cv::normalize(depth_mat, binary_depth, 0, 255, cv::NORM_MINMAX, CV_8UC1);

    for (int img_idx = 0; img_idx < 640 * 576; img_idx++)
    {
        const float depth_value = (float)(depth_mat.at<ushort>(img_idx) / 1000.0f);

        //DEPTH_CLIPPING
        if (depth_value > 1.9f)
            depth_mat.at<ushort>(img_idx) = 0;

        binary_depth.data[img_idx] = (depth_mat.at<ushort>(img_idx) > 0) ? 255 : 0;
    }
    
    /*
    for (int y = 0; y < 576; y++) {
        ushort* depth_pointer = depth_mat.ptr<ushort>(y);
        for (int x = 0; x < 640; x++) {
            const float depth_value = (float)(depth_pointer[x] / 1000.0f);
            //DEPTH_CLIPPING
            if (depth_value > 2.0f)
                depth_pointer[x] = 0;
            binary_depth.data[y*640+x] = (depth_pointer[x] > 0) ? 255 : 0;
        }
    }
    */
    //DIST_L2 - 유클리디안 거리 측정 (속도 문제로 변경), DIST_C
    cv::distanceTransform(binary_depth, dist_mat, cv::DIST_L2, 0, CV_32F);
   
    /*
    cv::imshow("aa", binary_depth);
    cv::waitKey(0);
    cv::imwrite("distmap.png", dist_mat);
    */

    //binary_depth.convertTo(dist_mat, CV_32F);
    return dist_mat;
}

cv::Mat Window3dWrapper::create_depth_mat(cv::Mat dist_mat, cv::Mat depth_mat) {


    for (int img_idx = 0; img_idx < 640 * 576; img_idx++)
    {
        const float distance_value = dist_mat.at<float>(img_idx);

        //OUTLIER_REMOVAL
        if (distance_value < 0) {
            depth_mat.at<ushort>(img_idx) = 0;
        }
    }
    
    //clock_t start = clock();


    /*
    for (int y = 0; y < 576; y++) {
        float* dist_pointer = dist_mat.ptr<float>(y);
        ushort* depth_pointer = depth_mat.ptr<ushort>(y);
        for (int x = 0; x < 640; x++) {
            const float distance_value = (float)(dist_pointer[x]);
            //OUTLIER_REMOVAL
            if (distance_value < 7.0f)
                depth_mat.data[x] = 0;
        }
    }
    */
    //clock_t end = clock(); // 코드가 끝난 시간 저장
    //printf("buffer : %lf\n", (double)(end - start) / CLOCKS_PER_SEC);

    /*
    cv::imshow("aa", depth_mat);
    cv::waitKey(0);
    */

    return depth_mat;
}


void Window3dWrapper::UpdatePointClouds_UV(k4a_image_t depthImage, k4a_image_t colorImage, int num, std::vector<float> extrinsic)

{

    m_pointCloudUpdated = true;
    int16_t* pointCloudImageBuffer = nullptr;

    const uint8_t* color_image_data = nullptr;

    if (num == 0) {
        m_pointClouds.reserve(50000);


        VERIFY(k4a_transformation_color_image_to_depth_camera(m_transformationHandle,
            depthImage,
            colorImage,
            transformed_color_image), "Transform color image to depth 1 camera failed!");

        VERIFY(k4a_transformation_depth_image_to_point_cloud(m_transformationHandle,
            depthImage,
            K4A_CALIBRATION_TYPE_DEPTH,
            m_pointCloudImage), "Transform depth image to point clouds failed!");

        pointCloudImageBuffer = (int16_t*)k4a_image_get_buffer(m_pointCloudImage);
        color_image_data = (uint8_t*)k4a_image_get_buffer(transformed_color_image);
    }
    if (num == 1) {
        m_pointClouds2.reserve(20000);

        VERIFY(k4a_transformation_color_image_to_depth_camera(m_transformationHandle2,
            depthImage,
            colorImage,
            transformed_color_image2), "Transform color image to depth 2 camera failed!");
     
        VERIFY(k4a_transformation_depth_image_to_point_cloud(m_transformationHandle2,
            depthImage,
            K4A_CALIBRATION_TYPE_DEPTH,
            m_pointCloudImage2), "Transform depth image to point clouds failed!");

        pointCloudImageBuffer = (int16_t*)k4a_image_get_buffer(m_pointCloudImage2);
        color_image_data = (uint8_t*)k4a_image_get_buffer(transformed_color_image2);
    }
    if (num == 2) {
        m_pointClouds3.reserve(20000);

        VERIFY(k4a_transformation_color_image_to_depth_camera(m_transformationHandle3,
            depthImage,
            colorImage,
            transformed_color_image3), "Transform color image to depth 3 camera failed!");

        VERIFY(k4a_transformation_depth_image_to_point_cloud(m_transformationHandle3,
            depthImage,
            K4A_CALIBRATION_TYPE_DEPTH,
            m_pointCloudImage3), "Transform depth image to point clouds failed!");


        pointCloudImageBuffer = (int16_t*)k4a_image_get_buffer(m_pointCloudImage3);
        color_image_data = (uint8_t*)k4a_image_get_buffer(transformed_color_image3);
    }
    if (num == 3) {
        m_pointClouds4.reserve(20000);

        VERIFY(k4a_transformation_color_image_to_depth_camera(m_transformationHandle4,
            depthImage,
            colorImage,
            transformed_color_image4), "Transform color image to depth 4 camera failed!");

        VERIFY(k4a_transformation_depth_image_to_point_cloud(m_transformationHandle4,
            depthImage,
            K4A_CALIBRATION_TYPE_DEPTH,
            m_pointCloudImage4), "Transform depth image to point clouds failed!");

        pointCloudImageBuffer = (int16_t*)k4a_image_get_buffer(m_pointCloudImage4);
        color_image_data = (uint8_t*)k4a_image_get_buffer(transformed_color_image4);
    }
    if (num == 4) {
        m_pointClouds5.reserve(20000);

        VERIFY(k4a_transformation_color_image_to_depth_camera(m_transformationHandle5,
            depthImage,
            colorImage,
            transformed_color_image5), "Transform color image to depth 5 camera failed!");

        VERIFY(k4a_transformation_depth_image_to_point_cloud(m_transformationHandle5,
            depthImage,
            K4A_CALIBRATION_TYPE_DEPTH,
            m_pointCloudImage5), "Transform depth image to point clouds failed!");

        pointCloudImageBuffer = (int16_t*)k4a_image_get_buffer(m_pointCloudImage5);
        color_image_data = (uint8_t*)k4a_image_get_buffer(transformed_color_image5);
    }
    if (num == 5) {
        m_pointClouds6.reserve(20000);

        VERIFY(k4a_transformation_color_image_to_depth_camera(m_transformationHandle6,
            depthImage,
            colorImage,
            transformed_color_image6), "Transform color image to depth 5 camera failed!");

        VERIFY(k4a_transformation_depth_image_to_point_cloud(m_transformationHandle6,
            depthImage,
            K4A_CALIBRATION_TYPE_DEPTH,
            m_pointCloudImage6), "Transform depth image to point clouds failed!");

        pointCloudImageBuffer = (int16_t*)k4a_image_get_buffer(m_pointCloudImage6);
        color_image_data = (uint8_t*)k4a_image_get_buffer(transformed_color_image6);
    }
    if (num == 6) {

        m_pointClouds7.reserve(20000);

        VERIFY(k4a_transformation_color_image_to_depth_camera(m_transformationHandle7,
            depthImage,
            colorImage,
            transformed_color_image7), "Transform color image to depth 6 camera failed!");

        VERIFY(k4a_transformation_depth_image_to_point_cloud(m_transformationHandle7,
            depthImage,
            K4A_CALIBRATION_TYPE_DEPTH,
            m_pointCloudImage7), "Transform depth image to point clouds failed!");

        pointCloudImageBuffer = (int16_t*)k4a_image_get_buffer(m_pointCloudImage7);
        color_image_data = (uint8_t*)k4a_image_get_buffer(transformed_color_image7);
    }
    if (num == 7) {
        m_pointClouds8.reserve(20000);

        VERIFY(k4a_transformation_color_image_to_depth_camera(m_transformationHandle8,
            depthImage,
            colorImage,
            transformed_color_image8), "Transform color image to depth 8 camera failed!");

        VERIFY(k4a_transformation_depth_image_to_point_cloud(m_transformationHandle8,
            depthImage,
            K4A_CALIBRATION_TYPE_DEPTH,
            m_pointCloudImage8), "Transform depth image to point clouds failed!");

        pointCloudImageBuffer = (int16_t*)k4a_image_get_buffer(m_pointCloudImage8);
        color_image_data = (uint8_t*)k4a_image_get_buffer(transformed_color_image8);
    }

    int width = 640;
    int height = 576;
    float u = 0.0f;
    float v = 0.0f;

    int point_count = 0;
    int h = 0;

    float x_r1 = 0;
    float y_r1 = 0;
    float z_r1 = 0;

    float x_r2 = 0;
    float y_r2 = 0;
    float z_r2 = 0;

    float x_r3 = 0;
    float y_r3 = 0;
    float z_r3 = 0;

    float x_t1 = 0;
    float y_t1 = 0;
    float z_t1 = 0;

    int k = num * 15;

    x_r1 = extrinsic[num + k];
    y_r1 = extrinsic[num + 1 + k];
    z_r1 = extrinsic[num + 2 + k];
    x_t1 = extrinsic[num + 3 + k];

    x_r2 = extrinsic[num + 4 + k];
    y_r2 = extrinsic[num + 5 + k];
    z_r2 = extrinsic[num + 6 + k];
    y_t1 = extrinsic[num + 7 + k];

    x_r3 = extrinsic[num + 8 + k];
    y_r3 = extrinsic[num + 9 + k];
    z_r3 = extrinsic[num + 10 + k];
    z_t1 = extrinsic[num + 11 + k];

    printf("%d\n", num);


//#pragma omp parallel
    {
        for (int h = 0; h < 576; h++)
        {

//#pragma omp for 
            for (int w = 0; w < 640; w++)
            {

                //printf("%d %d\n", w, h);

                int pixelIndex = h * 640 + w;


                float px = static_cast<float>(pointCloudImageBuffer[3 * pixelIndex + 0]);
                float py = static_cast<float>(pointCloudImageBuffer[3 * pixelIndex + 1]);
                float pz = static_cast<float>(pointCloudImageBuffer[3 * pixelIndex + 2]);

                // When the point cloud is invalid, the z-depth value is 0.s
                if (pz == 0)
                {
                    continue;
                }
                // When the point cloud is invalid, the z-depth value is 0.s
                //if (pz > 1750.0f)
                //{
                //    continue;
                //}

                //if (pz > 1950.0f)
                //{
                //    continue;
                //}
                
                //if (py > 545.0f || py < -1200.0f)
                //{
                //    continue;
                //}
                
                float ax = x_r1 * px + y_r1 * py + z_r1 * pz + x_t1;
                float ay = x_r2 * px + y_r2 * py + z_r2 * pz + y_t1;
                float az = x_r3 * px + y_r3 * py + z_r3 * pz + z_t1;

                k4a_float3_t position = { ax,ay,az };
                //k4a_float3_t position = { px,py,pz };

                u = (float)w / 640.0f;


                if (num == 0) {
                    v = ((float)h / 576.0f) * 0.125f;
                }
                if (num == 1) {
                    v = ((float)h / 576.0f) * 0.125f + 0.125f;
                }
                if (num == 2) {
                    v = ((float)h / 576.0f) * 0.125f + 0.25f;
                }
                if (num == 3) {
                    v = ((float)h / 576.0f) * 0.125f + 0.375f;
                }
                if (num == 4) {
                    v = ((float)h / 576.0f) * 0.125f + 0.5f;
                }
                if (num == 5) {
                    v = ((float)h / 576.0f) * 0.125f + 0.625f;
                }
                if (num == 6) {
                    v = ((float)h / 576.0f) * 0.125f + 0.75f;
                }
                if (num == 7) {
                    v = ((float)h / 576.0f) * 0.125f + 0.875;
                }

                float r = (float)color_image_data[4 * pixelIndex + 2] / 256.0f;
                float g = (float)color_image_data[4 * pixelIndex + 1] / 256.0f;
                float b = (float)color_image_data[4 * pixelIndex + 0] / 256.0f;

                if (r == 0 && g == 0 && b == 0) {
                    continue;
                }

                linmath::vec2 uv = { u, v };
                linmath::vec3 positionInMeter;
                linmath::vec4 color = { r, g, b, 0.2f };

                ConvertMillimeterToMeter(position, positionInMeter);
                Visualization::PointCloudVertex pointCloud;

                linmath::vec3_copy(pointCloud.Position, positionInMeter);
                linmath::vec2_copy(pointCloud.UV, uv);
                linmath::vec4_copy(pointCloud.Color, color);

                if (num == 0) {
                    m_pointClouds.push_back(pointCloud);
                }
                if (num == 1) {
                    m_pointClouds2.push_back(pointCloud);
                }
                if (num == 2) {
                    m_pointClouds3.push_back(pointCloud);
                }
                if (num == 3) {
                    m_pointClouds4.push_back(pointCloud);
                }
                if (num == 4) {
                    m_pointClouds5.push_back(pointCloud);
                }
                if (num == 5) {
                    m_pointClouds6.push_back(pointCloud);
                }
                if (num == 6) {
                    m_pointClouds7.push_back(pointCloud);
                }
                if (num == 7) {
                    m_pointClouds8.push_back(pointCloud);
                }

            }
        }

        //printf("7p size: %d\n", m_pointClouds7.size());
        //printf("8p size: %d\n", m_pointClouds8.size());
    }


    if (num == 0) {
        UpdateColorBuffer(transformed_color_image);
    }
    else if (num == 1) {
        UpdateColorBuffer2(transformed_color_image2);
    }
    else if (num == 2) {
        UpdateColorBuffer3(transformed_color_image3);
    }
    else if (num == 3) {
        UpdateColorBuffer4(transformed_color_image4);
    }
    else if (num == 4) {
        UpdateColorBuffer5(transformed_color_image5);
    }
    else if (num == 5) {
        UpdateColorBuffer6(transformed_color_image6);
    }
    else if (num == 6) {
        UpdateColorBuffer7(transformed_color_image7);

        //uint8_t* buffer = k4a_image_get_buffer(depthImage);
        //cv::Mat depth_map = cv::Mat(576, 640, CV_16UC1, (void*)buffer, cv::Mat::AUTO_STEP);
        //uint8_t* buffer2 = k4a_image_get_buffer(transformed_color_image7);
        //cv::Mat color_map = cv::Mat(576, 640, CV_8UC4, (void*)buffer2, cv::Mat::AUTO_STEP);
        //
        //cv::imwrite("frame-000007.depth.png", depth_map);
        //cv::imwrite("frame-000007.color.bmp", color_map);
    }
    else if (num == 7) {
        UpdateColorBuffer8(transformed_color_image8);

        //uint8_t* buffer = k4a_image_get_buffer(depthImage);
        //cv::Mat depth_map = cv::Mat(576, 640, CV_16UC1, (void*)buffer, cv::Mat::AUTO_STEP);
        //uint8_t* buffer2 = k4a_image_get_buffer(transformed_color_image8);
        //cv::Mat color_map = cv::Mat(576, 640, CV_8UC4, (void*)buffer2, cv::Mat::AUTO_STEP);
        //
        //cv::imwrite("frame-000008.depth.png", depth_map);
        //cv::imwrite("frame-000008.color.bmp", color_map);

    }
}




void Window3dWrapper::CleanJointsAndBones()
{
    m_window3d.CleanJointsAndBones();
}

void Window3dWrapper::AddJoint(k4a_float3_t position, k4a_quaternion_t orientation, Color color, int num, std::vector<float> extrinsic)
{
    linmath::vec3 jointPositionInMeter;
    ConvertMillimeterToMeter(position, jointPositionInMeter);

    float x_r1 = 0;
    float y_r1 = 0;
    float z_r1 = 0;

    float x_r2 = 0;
    float y_r2 = 0;
    float z_r2 = 0;

    float x_r3 = 0;
    float y_r3 = 0;
    float z_r3 = 0;

    float x_t1 = 0;
    float y_t1 = 0;
    float z_t1 = 0;
     
    int k = num * 15;

    x_r1 = extrinsic[num + k];
    y_r1 = extrinsic[num + 1 + k];
    z_r1 = extrinsic[num + 2 + k];
    x_t1 = extrinsic[num + 3 + k];

    x_r2 = extrinsic[num + 4 + k];
    y_r2 = extrinsic[num + 5 + k];
    z_r2 = extrinsic[num + 6 + k];
    y_t1 = extrinsic[num + 7 + k];

    x_r3 = extrinsic[num + 8 + k];
    y_r3 = extrinsic[num + 9 + k];
    z_r3 = extrinsic[num + 10 + k];
    z_t1 = extrinsic[num + 11 + k];

    float px = jointPositionInMeter[0];
    float py = jointPositionInMeter[1];
    float pz = jointPositionInMeter[2];

    jointPositionInMeter[0] = x_r1 * px + y_r1 * py + z_r1 * pz + (x_t1 / 1000);
    jointPositionInMeter[1] = x_r2 * px + y_r2 * py + z_r2 * pz + (y_t1 / 1000);
    jointPositionInMeter[2] = x_r3 * px + y_r3 * py + z_r3 * pz + (z_t1 / 1000);

    m_window3d.AddJoint({
        {jointPositionInMeter[0], jointPositionInMeter[1], jointPositionInMeter[2]},
        {orientation.v[0], orientation.v[1], orientation.v[2], orientation.v[3]},
        {color.r, color.g, color.b, color.a} });
}

void Window3dWrapper::AddBone(k4a_float3_t joint1Position, k4a_float3_t joint2Position, Color color, int num, std::vector<float> extrinsic)
{
    linmath::vec3 joint1PositionInMeter;
    ConvertMillimeterToMeter(joint1Position, joint1PositionInMeter);
    linmath::vec3 joint2PositionInMeter;
    ConvertMillimeterToMeter(joint2Position, joint2PositionInMeter);

    float x_r1 = 0;
    float y_r1 = 0;
    float z_r1 = 0;

    float x_r2 = 0;
    float y_r2 = 0;
    float z_r2 = 0;

    float x_r3 = 0;
    float y_r3 = 0;
    float z_r3 = 0;

    float x_t1 = 0;
    float y_t1 = 0;
    float z_t1 = 0;

    int k = num * 15;

    x_r1 = extrinsic[num + k];
    y_r1 = extrinsic[num + 1 + k];
    z_r1 = extrinsic[num + 2 + k];
    x_t1 = extrinsic[num + 3 + k];

    x_r2 = extrinsic[num + 4 + k];
    y_r2 = extrinsic[num + 5 + k];
    z_r2 = extrinsic[num + 6 + k];
    y_t1 = extrinsic[num + 7 + k];

    x_r3 = extrinsic[num + 8 + k];
    y_r3 = extrinsic[num + 9 + k];
    z_r3 = extrinsic[num + 10 + k];
    z_t1 = extrinsic[num + 11 + k];

    float px = joint1PositionInMeter[0];
    float py = joint1PositionInMeter[1];
    float pz = joint1PositionInMeter[2];

    joint1PositionInMeter[0] = x_r1 * px + y_r1 * py + z_r1 * pz + (x_t1 / 1000);
    joint1PositionInMeter[1] = x_r2 * px + y_r2 * py + z_r2 * pz + (y_t1 / 1000);
    joint1PositionInMeter[2] = x_r3 * px + y_r3 * py + z_r3 * pz + (z_t1 / 1000);

    float px2 = joint2PositionInMeter[0];
    float py2 = joint2PositionInMeter[1];
    float pz2 = joint2PositionInMeter[2];

    joint2PositionInMeter[0] = x_r1 * px2 + y_r1 * py2 + z_r1 * pz2 + (x_t1 / 1000);
    joint2PositionInMeter[1] = x_r2 * px2 + y_r2 * py2 + z_r2 * pz2 + (y_t1 / 1000);
    joint2PositionInMeter[2] = x_r3 * px2 + y_r3 * py2 + z_r3 * pz2 + (z_t1 / 1000);


    Visualization::Bone bone;
    linmath::vec4_copy(bone.Joint1Position, joint1PositionInMeter);
    linmath::vec4_copy(bone.Joint2Position, joint2PositionInMeter);
    bone.Color[0] = color.r;
    bone.Color[1] = color.g;
    bone.Color[2] = color.b;
    bone.Color[3] = color.a;

    m_window3d.AddBone(bone);
}

void Window3dWrapper::AddBody(const k4abt_body_t& body, Color color)
{
    Color lowConfidenceColor = color;
    lowConfidenceColor.a = color.a / 4;

    for (int joint = 0; joint < static_cast<int>(K4ABT_JOINT_COUNT); joint++)
    {
        if (body.skeleton.joints[joint].confidence_level >= K4ABT_JOINT_CONFIDENCE_LOW)
        {
            const k4a_float3_t& jointPosition = body.skeleton.joints[joint].position;
            const k4a_quaternion_t& jointOrientation = body.skeleton.joints[joint].orientation;

            //AddJoint(
            //    jointPosition,
            //    jointOrientation,
            //    body.skeleton.joints[joint].confidence_level >= K4ABT_JOINT_CONFIDENCE_MEDIUM ? color : lowConfidenceColor
            //    
            //);
        }
    }

    for (size_t boneIdx = 0; boneIdx < g_boneList.size(); boneIdx++)
    {
        //k4abt_joint_id_t joint1 = g_boneList[boneIdx].first;
        //k4abt_joint_id_t joint2 = g_boneList[boneIdx].second;
        //
        //if (body.skeleton.joints[joint1].confidence_level >= K4ABT_JOINT_CONFIDENCE_LOW &&
        //    body.skeleton.joints[joint2].confidence_level >= K4ABT_JOINT_CONFIDENCE_LOW)
        //{
        //    bool confidentBone = body.skeleton.joints[joint1].confidence_level >= K4ABT_JOINT_CONFIDENCE_MEDIUM &&
        //        body.skeleton.joints[joint2].confidence_level >= K4ABT_JOINT_CONFIDENCE_MEDIUM;
        //    const k4a_float3_t& joint1Position = body.skeleton.joints[joint1].position;
        //    const k4a_float3_t& joint2Position = body.skeleton.joints[joint2].position;
        //
        //    AddBone(joint1Position, joint2Position, confidentBone ? color : lowConfidenceColor);
        //}
    }
}

void Window3dWrapper::Render()
{
    if (m_pointCloudUpdated || m_pointClouds.size() != 0)
    {
        m_window3d.UpdatePointClouds(m_pointClouds.data(), (uint32_t)m_pointClouds.size(), m_depthWidth, m_depthHeight);
      
        m_pointClouds.clear();
        m_faceIndices.clear();
        m_pointCloudUpdated = false;
    }

    m_window3d.Render();
}


void Window3dWrapper::Render2()
{
    //m_window3d.Render2();
}

void Window3dWrapper::SetWindowPosition(int xPos, int yPos)
{
    m_window3d.SetWindowPosition(xPos, yPos);
}


void Window3dWrapper::SetLayout3d(Visualization::Layout3d layout3d)
{
    m_window3d.SetLayout3d(layout3d);
}

void Window3dWrapper::SetJointFrameVisualization(bool enableJointFrameVisualization)
{
    Visualization::SkeletonRenderMode skeletonRenderMode = enableJointFrameVisualization ?
        Visualization::SkeletonRenderMode::SkeletonOverlayWithJointFrame : Visualization::SkeletonRenderMode::SkeletonOverlay;

    m_window3d.SetSkeletonRenderMode(skeletonRenderMode);
}

void Window3dWrapper::SetFloorRendering(bool enableFloorRendering, float floorPositionX, float floorPositionY, float floorPositionZ)
{
    linmath::vec3 position = { floorPositionX, floorPositionY, floorPositionZ };
    m_window3d.SetFloorRendering(enableFloorRendering, position, {1.f, 0.f, 0.f, 0.f});
}

void Window3dWrapper::InitializeCalibration(const k4a_calibration_t& sensorCalibration, int num)
{

    m_depthWidth = 640;
    m_depthHeight = 576;
    

    if(num==0){
        if (m_transformationHandle == nullptr)
        {
            m_transformationHandle = k4a_transformation_create(&sensorCalibration);

            if (m_pointCloudImage == nullptr)
            {
                VERIFY(k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
                    m_depthWidth,
                    m_depthHeight,
                    m_depthWidth * 3 * (int)sizeof(int16_t),
                    &m_pointCloudImage), "Create Point Cloud Image failed!");
            }

            if (transformed_color_image == nullptr)
            {
                VERIFY(k4a_image_create(K4A_IMAGE_FORMAT_COLOR_BGRA32,
                    m_depthWidth,
                    m_depthHeight,
                    m_depthWidth * 4 * (int)sizeof(uint8_t),
                    &transformed_color_image), "Create Color Image failed!");
            }

        }
    }
    if (num == 1) {
        if (m_transformationHandle2 == nullptr)
        {
            m_transformationHandle2 = k4a_transformation_create(&sensorCalibration);

            if (m_pointCloudImage2 == nullptr)
            {
                VERIFY(k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
                    m_depthWidth,
                    m_depthHeight,
                    m_depthWidth * 3 * (int)sizeof(int16_t),
                    &m_pointCloudImage2), "Create Point Cloud Image failed!");
            }

            if (transformed_color_image2 == nullptr)
            {
                VERIFY(k4a_image_create(K4A_IMAGE_FORMAT_COLOR_BGRA32,
                    m_depthWidth,
                    m_depthHeight,
                    m_depthWidth * 4 * (int)sizeof(uint8_t),
                    &transformed_color_image2), "Create Color Image failed!");
            }
        }
    }
    if (num == 2) {
        if (m_transformationHandle3 == nullptr)
        {
            m_transformationHandle3 = k4a_transformation_create(&sensorCalibration);

            if (m_pointCloudImage3 == nullptr)
            {
                VERIFY(k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
                    m_depthWidth,
                    m_depthHeight,
                    m_depthWidth * 3 * (int)sizeof(int16_t),
                    &m_pointCloudImage3), "Create Point Cloud Image failed!");
            }

            if (transformed_color_image3 == nullptr)
            {
                VERIFY(k4a_image_create(K4A_IMAGE_FORMAT_COLOR_BGRA32,
                    m_depthWidth,
                    m_depthHeight,
                    m_depthWidth * 4 * (int)sizeof(uint8_t),
                    &transformed_color_image3), "Create Color Image failed!");
            }
        }
    }
    if (num == 3) {
        if (m_transformationHandle4 == nullptr)
        {
            m_transformationHandle4 = k4a_transformation_create(&sensorCalibration);

            if (m_pointCloudImage4 == nullptr)
            {
                VERIFY(k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
                    m_depthWidth,
                    m_depthHeight,
                    m_depthWidth * 3 * (int)sizeof(int16_t),
                    &m_pointCloudImage4), "Create Point Cloud Image failed!");
            }

            if (transformed_color_image4 == nullptr)
            {
                VERIFY(k4a_image_create(K4A_IMAGE_FORMAT_COLOR_BGRA32,
                    m_depthWidth,
                    m_depthHeight,
                    m_depthWidth * 4 * (int)sizeof(uint8_t),
                    &transformed_color_image4), "Create Color Image failed!");
            }
        }
    }
    if (num == 4) {
        if (m_transformationHandle5 == nullptr)
        {
            m_transformationHandle5 = k4a_transformation_create(&sensorCalibration);

            if (m_pointCloudImage5 == nullptr)
            {
                VERIFY(k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
                    m_depthWidth,
                    m_depthHeight,
                    m_depthWidth * 3 * (int)sizeof(int16_t),
                    &m_pointCloudImage5), "Create Point Cloud Image failed!");
            }

            if (transformed_color_image5 == nullptr)
            {
                VERIFY(k4a_image_create(K4A_IMAGE_FORMAT_COLOR_BGRA32,
                    m_depthWidth,
                    m_depthHeight,
                    m_depthWidth * 4 * (int)sizeof(uint8_t),
                    &transformed_color_image5), "Create Color Image failed!");
            }
        }
    }
    if (num == 5) {
        if (m_transformationHandle6 == nullptr)
        {
            m_transformationHandle6 = k4a_transformation_create(&sensorCalibration);

            if (m_pointCloudImage6 == nullptr)
            {
                VERIFY(k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
                    m_depthWidth,
                    m_depthHeight,
                    m_depthWidth * 3 * (int)sizeof(int16_t),
                    &m_pointCloudImage6), "Create Point Cloud Image failed!");
            }

            if (transformed_color_image6 == nullptr)
            {
                VERIFY(k4a_image_create(K4A_IMAGE_FORMAT_COLOR_BGRA32,
                    m_depthWidth,
                    m_depthHeight,
                    m_depthWidth * 4 * (int)sizeof(uint8_t),
                    &transformed_color_image6), "Create Color Image failed!");
            }
        }
    }
    if (num == 6) {
        if (m_transformationHandle7 == nullptr)
        {
            m_transformationHandle7 = k4a_transformation_create(&sensorCalibration);

            if (m_pointCloudImage7 == nullptr)
            {
                VERIFY(k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
                    m_depthWidth,
                    m_depthHeight,
                    m_depthWidth * 3 * (int)sizeof(int16_t),
                    &m_pointCloudImage7), "Create Point Cloud Image failed!");
            }

            if (transformed_color_image7 == nullptr)
            {
                VERIFY(k4a_image_create(K4A_IMAGE_FORMAT_COLOR_BGRA32,
                    m_depthWidth,
                    m_depthHeight,
                    m_depthWidth * 4 * (int)sizeof(uint8_t),
                    &transformed_color_image7), "Create Color Image failed!");
            }
        }
    }
    if (num == 7) {
        if (m_transformationHandle8 == nullptr)
        {
            m_transformationHandle8 = k4a_transformation_create(&sensorCalibration);

            if (m_pointCloudImage8 == nullptr)
            {
                VERIFY(k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
                    m_depthWidth,
                    m_depthHeight,
                    m_depthWidth * 3 * (int)sizeof(int16_t),
                    &m_pointCloudImage8), "Create Point Cloud Image failed!");
            }

            if (transformed_color_image8 == nullptr)
            {
                VERIFY(k4a_image_create(K4A_IMAGE_FORMAT_COLOR_BGRA32,
                    m_depthWidth,
                    m_depthHeight,
                    m_depthWidth * 4 * (int)sizeof(uint8_t),
                    &transformed_color_image8), "Create Color Image failed!");
            }
        }
    }
}

void Window3dWrapper::BlendBodyColor(linmath::vec4 color, Color bodyColor)
{
    float darkenRatio = 1.0f;
    float instanceAlpha = 1.0f;

    color[0] = bodyColor.r * instanceAlpha + color[0] * darkenRatio;
    color[1] = bodyColor.g * instanceAlpha + color[1] * darkenRatio;
    color[2] = bodyColor.b * instanceAlpha + color[2] * darkenRatio;
}


void Window3dWrapper::UpdateColorBuffer(k4a_image_t colorFrame)
{
    int width = 640;
    int height = 576;
    uint8_t* colorFrameBuffer = (uint8_t*)k4a_image_get_buffer(colorFrame);
    m_colorBuffer.assign(colorFrameBuffer, colorFrameBuffer + width * 4 * height);
}
void Window3dWrapper::UpdateColorBuffer2(k4a_image_t colorFrame)
{
    int width = 640;
    int height = 576;
    uint8_t* colorFrameBuffer = (uint8_t*)k4a_image_get_buffer(colorFrame);
    m_colorBuffer2.assign(colorFrameBuffer, colorFrameBuffer + width * 4 * height);
}
void Window3dWrapper::UpdateColorBuffer3(k4a_image_t colorFrame)
{
    int width = 640;
    int height = 576;
    uint8_t* colorFrameBuffer = (uint8_t*)k4a_image_get_buffer(colorFrame);
    m_colorBuffer3.assign(colorFrameBuffer, colorFrameBuffer + width * 4 * height);
 }

void Window3dWrapper::UpdateColorBuffer4(k4a_image_t colorFrame)
{
    int width = 640;
    int height = 576;
    uint8_t* colorFrameBuffer = (uint8_t*)k4a_image_get_buffer(colorFrame);
    m_colorBuffer4.assign(colorFrameBuffer, colorFrameBuffer + width * 4 * height);
}

void Window3dWrapper::UpdateColorBuffer5(k4a_image_t colorFrame)
{
    int width = 640;
    int height = 576;
    uint8_t* colorFrameBuffer = (uint8_t*)k4a_image_get_buffer(colorFrame);
    m_colorBuffer5.assign(colorFrameBuffer, colorFrameBuffer + width * 4 * height);
}

void Window3dWrapper::UpdateColorBuffer6(k4a_image_t colorFrame)
{
    int width = 640;
    int height = 576;
    uint8_t* colorFrameBuffer = (uint8_t*)k4a_image_get_buffer(colorFrame);
    m_colorBuffer6.assign(colorFrameBuffer, colorFrameBuffer + width * 4 * height);
}

void Window3dWrapper::UpdateColorBuffer7(k4a_image_t colorFrame)
{
    int width = 640;
    int height = 576;
    uint8_t* colorFrameBuffer = (uint8_t*)k4a_image_get_buffer(colorFrame);
    m_colorBuffer7.assign(colorFrameBuffer, colorFrameBuffer + width * 4 * height);
}

void Window3dWrapper::UpdateColorBuffer8(k4a_image_t colorFrame)
{
    int width = 640;
    int height = 576;
    uint8_t* colorFrameBuffer = (uint8_t*)k4a_image_get_buffer(colorFrame);
    m_colorBuffer8.assign(colorFrameBuffer, colorFrameBuffer + width * 4 * height);
}

void Window3dWrapper::ConcateBuffer() {
    m_colorBuffer.insert(m_colorBuffer.end(), m_colorBuffer2.begin(), m_colorBuffer2.end());
    m_colorBuffer.insert(m_colorBuffer.end(), m_colorBuffer3.begin(), m_colorBuffer3.end());
    m_colorBuffer.insert(m_colorBuffer.end(), m_colorBuffer4.begin(), m_colorBuffer4.end());
    m_colorBuffer.insert(m_colorBuffer.end(), m_colorBuffer5.begin(), m_colorBuffer5.end());
    m_colorBuffer.insert(m_colorBuffer.end(), m_colorBuffer6.begin(), m_colorBuffer6.end());
    m_colorBuffer.insert(m_colorBuffer.end(), m_colorBuffer7.begin(), m_colorBuffer7.end());
    m_colorBuffer.insert(m_colorBuffer.end(), m_colorBuffer8.begin(), m_colorBuffer8.end());
}

void Window3dWrapper::ConcatePoint() {
    m_pointClouds.insert(m_pointClouds.end(), m_pointClouds2.begin(), m_pointClouds2.end());
    m_pointClouds.insert(m_pointClouds.end(), m_pointClouds3.begin(), m_pointClouds3.end());
    m_pointClouds.insert(m_pointClouds.end(), m_pointClouds4.begin(), m_pointClouds4.end());
    m_pointClouds.insert(m_pointClouds.end(), m_pointClouds5.begin(), m_pointClouds5.end());
    m_pointClouds.insert(m_pointClouds.end(), m_pointClouds6.begin(), m_pointClouds6.end());
    m_pointClouds.insert(m_pointClouds.end(), m_pointClouds7.begin(), m_pointClouds7.end());
    m_pointClouds.insert(m_pointClouds.end(), m_pointClouds8.begin(), m_pointClouds8.end());
    
    //m_faceIndices.reserve(500000);

    //for (int i = 0; i < m_pointClouds.size(); i += 4) {
    //    m_faceIndices.push_back(i);
    //    m_faceIndices.push_back(i + 1);
    //    m_faceIndices.push_back(i + 2);
    //    m_faceIndices.push_back(i + 3);
    //    //printf("%d,%d,%d,%d\n",i + 0, i + 1, i + 2, i + 3);
    //}
    //m_pointClouds.clear();
    m_pointClouds2.clear();
    m_pointClouds3.clear();
    m_pointClouds4.clear();
    m_pointClouds5.clear();
    m_pointClouds6.clear();
    m_pointClouds7.clear();
    m_pointClouds8.clear();
}

bool Window3dWrapper::CreateXYDepthTable(const k4a_calibration_t & sensorCalibration)
{
    int width = 640;
    int height = 576;
    
    m_xyDepthTable.resize(width * height);

    auto xyTablePtr = m_xyDepthTable.begin();

    k4a_float3_t pt3;
    for (int h = 0; h < height; h++)
        {
            for (int w = 0; w < width; w++)
            {
                k4a_float2_t pt = { static_cast<float>(w), static_cast<float>(h) };
                int valid = 0;
                k4a_result_t result = k4a_calibration_2d_to_3d(&sensorCalibration,
                    &pt,
                    1.f,
                    K4A_CALIBRATION_TYPE_DEPTH,
                    K4A_CALIBRATION_TYPE_DEPTH,
                    &pt3,
                    &valid);
                if (result != K4A_RESULT_SUCCEEDED)
                {
                    return false;
                }

                if (valid == 0)
                {
                    // Set the invalid xy table to be (0, 0)
                    xyTablePtr->x = 0.f;
                    xyTablePtr->y = 0.f;
                }
                else
                {
                    xyTablePtr->x = pt3.xyz.x;
                    xyTablePtr->y = pt3.xyz.y;
                }

                ++xyTablePtr;
            }
        }
    return true;
}

