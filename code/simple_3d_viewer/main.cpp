// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include <array>
#include <iostream>
#include <map>
#include <vector>

#include <time.h>
#include <omp.h>
#include <k4a/k4a.h>
#include <k4abt.h>

#include <BodyTrackingHelpers.h>
#include <Utilities.h>
#include <Window3dWrapper.h>
#include <opencv2/opencv.hpp>

void PrintUsage()
{
    printf("\nUSAGE: (k4abt_)simple_3d_viewer.exe SensorMode[NFOV_UNBINNED, WFOV_BINNED](optional) RuntimeMode[CPU](optional)\n");
    printf("  - SensorMode: \n");
    printf("      NFOV_UNBINNED (default) - Narraw Field of View Unbinned Mode [Resolution: 640x576; FOI: 75 degree x 65 degree]\n");
    printf("      WFOV_BINNED             - Wide Field of View Binned Mode [Resolution: 512x512; FOI: 120 degree x 120 degree]\n");
    printf("  - RuntimeMode: \n");
    printf("      CPU - Use the CPU only mode. It runs on machines without a GPU but it will be much slower\n");
    printf("e.g.   (k4abt_)simple_3d_viewer.exe WFOV_BINNED CPU\n");
    printf("e.g.   (k4abt_)simple_3d_viewer.exe CPU\n");
    printf("e.g.   (k4abt_)simple_3d_viewer.exe WFOV_BINNED\n");
}

void PrintAppUsage()
{
    printf("\n");
    printf(" Basic Navigation:\n\n");
    printf(" Rotate: Rotate the camera by moving the mouse while holding mouse left button\n");
    printf(" Pan: Translate the scene by holding Ctrl key and drag the scene with mouse left button\n");
    printf(" Zoom in/out: Move closer/farther away from the scene center by scrolling the mouse scroll wheel\n");
    printf(" Select Center: Center the scene based on a detected joint by right clicking the joint with mouse\n");
    printf("\n");
    printf(" Key Shortcuts\n\n");
    printf(" ESC: quit\n");
    printf(" h: help\n");
    printf(" b: body visualization mode\n");
    printf(" k: 3d window layout\n");
    printf("\n");
}

// Global State and Key Process Function
bool s_isRunning = true;
Visualization::Layout3d s_layoutMode = Visualization::Layout3d::OnlyMainView;
bool s_visualizeJointFrame = false;

int64_t ProcessKey(void* /*context*/, int key)
{
    // https://www.glfw.org/docs/latest/group__keys.html
    switch (key)
    {
        // Quit
    case GLFW_KEY_ESCAPE:
        s_isRunning = false;
        break;
    case GLFW_KEY_K:
        s_layoutMode = (Visualization::Layout3d)(((int)s_layoutMode + 1) % (int)Visualization::Layout3d::Count);
        break;
    case GLFW_KEY_B:
        s_visualizeJointFrame = !s_visualizeJointFrame;
        break;
    case GLFW_KEY_H:
        PrintAppUsage();
        break;
    }
    return 1;
}

int64_t CloseCallback(void* /*context*/)
{
    s_isRunning = false;
    return 1;
}

struct InputSettings
{
    k4a_depth_mode_t DepthCameraMode = K4A_DEPTH_MODE_NFOV_UNBINNED;
    bool CpuOnlyMode = false;
};

bool ParseInputSettingsFromArg(int argc, char** argv, InputSettings& inputSettings)
{
    for (int i = 1; i < argc; i++)
    {
        std::string inputArg(argv[i]);
        if (inputArg == std::string("NFOV_UNBINNED"))
        {
            inputSettings.DepthCameraMode = K4A_DEPTH_MODE_NFOV_UNBINNED;
        }
        else if (inputArg == std::string("WFOV_BINNED"))
        {
            inputSettings.DepthCameraMode = K4A_DEPTH_MODE_WFOV_2X2BINNED;
        }
        else if (inputArg == std::string("CPU"))
        {
            inputSettings.CpuOnlyMode = true;
        }
        else
        {
            return false;
        }
    }
    return true;
}


using namespace std;

std::vector<float> LoadMatrixFromFile(std::string filename, int M, int N) {
    std::vector<float> matrix;
    FILE* fp = fopen(filename.c_str(), "r");
    for (int i = 0; i < M * N; i++) {
        float tmp;
        int iret = fscanf(fp, "%f", &tmp);
        matrix.push_back(tmp);
    }
    fclose(fp);
    return matrix;
}

int main(int argc, char** argv)


{


    int only_image = 0;

    int idx = 1;
    int cam_idx = 1;

    int idx2 = 1;
    int cam_idx2 = 1;


    int iteration = 0;

    queue<vector<Visualization::PointCloudVertex>> point_cloud_data;
    queue<vector<Window3dWrapper::jointcopy>> joint_data;


    std::thread save_thread([&]() {
        while (true) {
            if (point_cloud_data.size() > 0) {
                char file_name[128];
                sprintf(file_name, "output/%d_%d_point_ori.ply", iteration, cam_idx);
                std::stringstream ss2;
    
                ss2 << "ply" << std::endl;
                ss2 << "format ascii 1.0" << std::endl;
                ss2 << "element vertex"
                    << " " << point_cloud_data.front().size() << std::endl;
    
                ss2 << "property float x" << std::endl;
                ss2 << "property float y" << std::endl;
                ss2 << "property float z" << std::endl;
                ss2 << "property uchar red" << std::endl;
                ss2 << "property uchar green" << std::endl;
                ss2 << "property uchar blue" << std::endl;
                ss2 << "end_header" << std::endl;
    
    
                for (int i = 0; i < point_cloud_data.front().size(); i++) {
                    ss2 << point_cloud_data.front()[i].Position[0] << " "
                        << point_cloud_data.front()[i].Position[1] << " "
                        << point_cloud_data.front()[i].Position[2] << " "
                        << (int)floor(point_cloud_data.front()[i].Color[0] * 256) << " "
                        << (int)floor(point_cloud_data.front()[i].Color[1] * 256) << " "
                        << (int)floor(point_cloud_data.front()[i].Color[2] * 256) << " "
                        << std::endl;
                }
    
                std::ofstream ofs_text2(file_name, std::ios::out | std::ios::app);
                ofs_text2.write(ss2.str().c_str(), (std::streamsize)ss2.str().length());
    
                point_cloud_data.pop();
                //joint_data.pop();
               
                cam_idx++;
    
                if (cam_idx > 8) {
                    idx++;
                    cam_idx = 1;
                }
            }
        }
        });

    
    std::thread save_thread_joint([&]() {
        while (true) {
            if (joint_data.size() > 0) {
                char file_name2[128];
                sprintf(file_name2, "output/%d_%d_joint_ori.ply", iteration, cam_idx2);
                std::stringstream ss;
    
                ss << "ply" << std::endl;
                ss << "format ascii 1.0" << std::endl;
                ss << "element vertex"
                    << " " << joint_data.front().size() << std::endl;
    
                ss << "property float x" << std::endl;
                ss << "property float y" << std::endl;
                ss << "property float z" << std::endl;
                ss << "property uchar idx" << std::endl;
                ss << "end_header" << std::endl;
    
    
                for (int i = 0; i < joint_data.front().size(); i++) {
                    ss << joint_data.front()[i].x << " "
                        << joint_data.front()[i].y << " "
                        << joint_data.front()[i].z << " "
                        << (int)joint_data.front()[i].num << " "
                        << std::endl;
                }
    
                std::ofstream ofs_text(file_name2, std::ios::out | std::ios::app);
                ofs_text.write(ss.str().c_str(), (std::streamsize)ss.str().length());
    
                joint_data.pop();
                //joint_data.pop();
               
                cam_idx2++;
    
                if (cam_idx2 > 8) {
                    idx2++;
                    cam_idx2 = 1;
                }
            }
        }
        });



    InputSettings inputSettings;
    if (!ParseInputSettingsFromArg(argc, argv, inputSettings))
    {
        PrintUsage();
        return -1;
    }
    PrintAppUsage();

    k4a_device_t device = nullptr;
    k4a_device_t device2 = nullptr;
    k4a_device_t device3 = nullptr;
    k4a_device_t device4 = nullptr;
    k4a_device_t device5 = nullptr;
    k4a_device_t device6 = nullptr;
    k4a_device_t device7 = nullptr;
    k4a_device_t device8 = nullptr;
    
    int update_is = 0;

    VERIFY(k4a_device_open(0, &device), "Open K4A 1 Device failed!");
    VERIFY(k4a_device_open(1, &device2), "Open K4A 2 Device failed!");
    VERIFY(k4a_device_open(2, &device3), "Open K4A 3 Device failed!");
    VERIFY(k4a_device_open(3, &device4), "Open K4A 4 Device failed!");
    VERIFY(k4a_device_open(4, &device5), "Open K4A 5 Device failed!");
    VERIFY(k4a_device_open(5, &device6), "Open K4A 6 Device failed!");
    VERIFY(k4a_device_open(6, &device7), "Open K4A 7 Device failed!");
    VERIFY(k4a_device_open(7, &device8), "Open K4A 8 Device failed!");
    

    k4a_device_configuration_t deviceConfig = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    deviceConfig.camera_fps = K4A_FRAMES_PER_SECOND_30;
    deviceConfig.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
    deviceConfig.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
    deviceConfig.color_resolution = K4A_COLOR_RESOLUTION_720P;
    //deviceConfig.wired_sync_mode = K4A_WIRED_SYNC_MODE_MASTER;
    deviceConfig.synchronized_images_only = true;
    deviceConfig.depth_delay_off_color_usec = 0;
    deviceConfig.disable_streaming_indicator = false;

    k4a_device_configuration_t deviceConfig2 = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    deviceConfig2.camera_fps = K4A_FRAMES_PER_SECOND_30;
    deviceConfig2.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
    deviceConfig2.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
    deviceConfig2.color_resolution = K4A_COLOR_RESOLUTION_720P;
    //deviceConfig2.wired_sync_mode = K4A_WIRED_SYNC_MODE_SUBORDINATE;
    deviceConfig2.synchronized_images_only = true;
    deviceConfig2.depth_delay_off_color_usec = 0;
    deviceConfig2.disable_streaming_indicator = false;


    VERIFY(k4a_device_start_cameras(device, &deviceConfig2), "Start K4A cameras failed!");
    VERIFY(k4a_device_start_cameras(device2, &deviceConfig2), "Start K4A cameras failed!");
    VERIFY(k4a_device_start_cameras(device3, &deviceConfig), "Start K4A cameras failed!");
    VERIFY(k4a_device_start_cameras(device4, &deviceConfig2), "Start K4A cameras failed!");
    VERIFY(k4a_device_start_cameras(device5, &deviceConfig2), "Start K4A cameras failed!");
    VERIFY(k4a_device_start_cameras(device6, &deviceConfig2), "Start K4A cameras failed!");
    VERIFY(k4a_device_start_cameras(device7, &deviceConfig2), "Start K4A cameras failed!");
    VERIFY(k4a_device_start_cameras(device8, &deviceConfig2), "Start K4A cameras failed!");


    // Get calibration information
    k4a_calibration_t sensorCalibration;
    k4a_calibration_t sensorCalibration2;
    k4a_calibration_t sensorCalibration3;
    k4a_calibration_t sensorCalibration4;    
    k4a_calibration_t sensorCalibration5;
    k4a_calibration_t sensorCalibration6;
    k4a_calibration_t sensorCalibration7;
    k4a_calibration_t sensorCalibration8;


    VERIFY(k4a_device_get_calibration(device, deviceConfig.depth_mode, deviceConfig.color_resolution, &sensorCalibration),
        "Get depth camera calibration failed!");
    VERIFY(k4a_device_get_calibration(device2, deviceConfig.depth_mode, deviceConfig.color_resolution, &sensorCalibration2),
        "Get depth camera calibration failed!");
    VERIFY(k4a_device_get_calibration(device3, deviceConfig.depth_mode, deviceConfig.color_resolution, &sensorCalibration3),
        "Get depth camera calibration failed!");
    VERIFY(k4a_device_get_calibration(device4, deviceConfig.depth_mode, deviceConfig.color_resolution, &sensorCalibration4),
        "Get depth camera calibration failed!");
    VERIFY(k4a_device_get_calibration(device5, deviceConfig.depth_mode, deviceConfig.color_resolution, &sensorCalibration5),
        "Get depth camera calibration failed!");
    VERIFY(k4a_device_get_calibration(device6, deviceConfig.depth_mode, deviceConfig.color_resolution, &sensorCalibration6),
        "Get depth camera calibration failed!");
    VERIFY(k4a_device_get_calibration(device7, deviceConfig.depth_mode, deviceConfig.color_resolution, &sensorCalibration7),
        "Get depth camera calibration failed!");
    VERIFY(k4a_device_get_calibration(device8, deviceConfig.depth_mode, deviceConfig.color_resolution, &sensorCalibration8),
        "Get depth camera calibration failed!");


    k4abt_tracker_t tracker = nullptr;
    k4abt_tracker_t tracker2 = nullptr;
    k4abt_tracker_t tracker3 = nullptr;
    k4abt_tracker_t tracker4 = nullptr;
    k4abt_tracker_t tracker5 = nullptr;
    k4abt_tracker_t tracker6 = nullptr;
    k4abt_tracker_t tracker7 = nullptr;
    k4abt_tracker_t tracker8 = nullptr;

    k4abt_tracker_configuration_t tracker_config = { K4ABT_SENSOR_ORIENTATION_DEFAULT,  // sensor_orientation
                                                      K4ABT_TRACKER_PROCESSING_MODE_GPU, // processing_mode
                                                      0 };
    k4abt_tracker_configuration_t tracker_config2 = { K4ABT_SENSOR_ORIENTATION_DEFAULT,  // sensor_orientation
                                                      K4ABT_TRACKER_PROCESSING_MODE_GPU, // processing_mode
                                                      0 };

    VERIFY(k4abt_tracker_create(&sensorCalibration, tracker_config, &tracker),
        "Body tracker initialization failed!");
    VERIFY(k4abt_tracker_create(&sensorCalibration2, tracker_config, &tracker2),
        "Body tracker initialization failed!");
    VERIFY(k4abt_tracker_create(&sensorCalibration3, tracker_config, &tracker3),
        "Body tracker initialization failed!");
    VERIFY(k4abt_tracker_create(&sensorCalibration4, tracker_config, &tracker4),
        "Body tracker initialization failed!");
    VERIFY(k4abt_tracker_create(&sensorCalibration5, tracker_config2, &tracker5),
        "Body tracker initialization failed!");
    VERIFY(k4abt_tracker_create(&sensorCalibration6, tracker_config2, &tracker6),
        "Body tracker initialization failed!");
    VERIFY(k4abt_tracker_create(&sensorCalibration7, tracker_config2, &tracker7),
        "Body tracker initialization failed!");
    VERIFY(k4abt_tracker_create(&sensorCalibration8, tracker_config2, &tracker8),
        "Body tracker initialization failed!");

    // Initialize the 3d window controller
    Window3dWrapper window3d;
    window3d.Create("3D Visualization", sensorCalibration, sensorCalibration2, sensorCalibration3, sensorCalibration4, sensorCalibration5, sensorCalibration6, sensorCalibration7, sensorCalibration8);
    window3d.SetCloseCallback(CloseCallback);
    window3d.SetKeyCallback(ProcessKey);
    //window3d.SetFloorRendering(1,0.f, 1.f, 1.f);

    //camera parameter
    std::vector<float> intrinsic;
    std::vector<float> extrinsic;

    // Location of folder camera pose files
    float intrinsics_parameter[3 * 3];
    float extrinsics_parameter[4 * 4];
    std::vector<float> extrinsics_parameter_vec;

    for (int cam_idx = 0; cam_idx < 8; cam_idx++) {
        
        //처음부터 맞출 때 기본 값
        extrinsic.push_back(1.0f);
        extrinsic.push_back(0.0f);
        extrinsic.push_back(0.0f);
        extrinsic.push_back(0.0f);  //T.x
        extrinsic.push_back(0.0f);
        extrinsic.push_back(1.0f);
        extrinsic.push_back(0.0f);
        extrinsic.push_back(0.0f);  //T.y
        extrinsic.push_back(0.0f);
        extrinsic.push_back(0.0f);
        extrinsic.push_back(1.0f);
        extrinsic.push_back(0.0f);  //T.z
        extrinsic.push_back(0.0f);
        extrinsic.push_back(0.0f);
        extrinsic.push_back(0.0f);
        extrinsic.push_back(1.0f);

        // 다시 맞출 때 matrix 불러오기, 프레임 넘버 수정할 것 
        // Read Extrinsic
        //std::string extrinsics_parameter_file = "344_" + to_string(cam_idx + 1) + "_matrix.txt";
        //extrinsics_parameter_vec = LoadMatrixFromFile(extrinsics_parameter_file, 4, 4);
        //
        //extrinsic.push_back((float)extrinsics_parameter_vec[0]);
        //extrinsic.push_back((float)extrinsics_parameter_vec[1]);
        //extrinsic.push_back((float)extrinsics_parameter_vec[2]);
        //extrinsic.push_back((float)extrinsics_parameter_vec[3]);  //T.x
        //extrinsic.push_back((float)extrinsics_parameter_vec[4]);
        //extrinsic.push_back((float)extrinsics_parameter_vec[5]);
        //extrinsic.push_back((float)extrinsics_parameter_vec[6]);
        //extrinsic.push_back((float)extrinsics_parameter_vec[7]);  //T.y
        //extrinsic.push_back((float)extrinsics_parameter_vec[8]);
        //extrinsic.push_back((float)extrinsics_parameter_vec[9]);
        //extrinsic.push_back((float)extrinsics_parameter_vec[10]);
        //extrinsic.push_back((float)extrinsics_parameter_vec[11]);  //T.z
        //extrinsic.push_back((float)extrinsics_parameter_vec[12]);
        //extrinsic.push_back((float)extrinsics_parameter_vec[13]);
        //extrinsic.push_back((float)extrinsics_parameter_vec[14]);
        //extrinsic.push_back((float)extrinsics_parameter_vec[15]);
    }

    struct calib_idx{
        int frame_num;
        int ref_num;
        int cam_num;
        float theta;
    };

    //std::vector<calib_idx> calib_order;
    //std::vector<calib_idx> calib_order_sort;


    std::vector<double> loss_value;

    for (int cam_idx = 0; cam_idx < 8; cam_idx++) {
        //다시 맞출 때 업데이트 고정
        //loss_value.push_back(0);

        //처음 부터 맞출 때
        loss_value.push_back(100.0f);
    }
    //front 맞출 때 
    //loss_value[3] = 0.08;
    //loss_value[4] = 0.08;
    //loss_value[5] = 0.08;
    //loss_value[7] = 0.08;

    //back 맞출 때 
    //loss_value[0] = 0.05;
    //loss_value[1] = 0.05;
    //loss_value[2] = 0.05;
    //loss_value[6] = 0.05;

	while (s_isRunning)
	{

        if (GetAsyncKeyState('A') & 0x8000)
        {
            loss_value[0] = 0.08;
            loss_value[1] = 0.08;
            loss_value[2] = 0.08;
            loss_value[3] = 0.08;
            loss_value[4] = 0.08;
            loss_value[5] = 0.08;
            loss_value[6] = 0.08;
            loss_value[7] = 0.08;
        }

        k4a_capture_t sensorCapture = nullptr;
        k4a_capture_t sensorCapture2 = nullptr;
        k4a_capture_t sensorCapture3 = nullptr;
        k4a_capture_t sensorCapture4 = nullptr;
        k4a_capture_t sensorCapture5 = nullptr;
        k4a_capture_t sensorCapture6 = nullptr;
        k4a_capture_t sensorCapture7 = nullptr;
        k4a_capture_t sensorCapture8 = nullptr;

        clock_t start = clock(); // 코드가 끝난 시간 저장


        k4abt_frame_t bodyFrame = nullptr;
        k4abt_frame_t bodyFrame2 = nullptr;
        k4abt_frame_t bodyFrame3 = nullptr;
        k4abt_frame_t bodyFrame4 = nullptr;
        k4abt_frame_t bodyFrame5 = nullptr;
        k4abt_frame_t bodyFrame6 = nullptr;
        k4abt_frame_t bodyFrame7 = nullptr;
        k4abt_frame_t bodyFrame8 = nullptr;


        k4a_wait_result_t popFrameResult; // timeout_in_ms is set to 0
        k4a_wait_result_t popFrameResult2; // timeout_in_ms is set to 0
        k4a_wait_result_t popFrameResult3; // timeout_in_ms is set to 0
        k4a_wait_result_t popFrameResult4; // timeout_in_ms is set to 0
        k4a_wait_result_t popFrameResult5; // timeout_in_ms is set to 0
        k4a_wait_result_t popFrameResult6; // timeout_in_ms is set to 0
        k4a_wait_result_t popFrameResult7; // timeout_in_ms is set to 0
        k4a_wait_result_t popFrameResult8; // timeout_in_ms is set to 0


#pragma omp parallel num_threads(8)
        {
#pragma omp sections
            {
#pragma omp section
                {
                    k4a_wait_result_t getCaptureResult = k4a_device_get_capture(device, &sensorCapture, 0); // timeout_in_ms is set to 0
                    if (getCaptureResult == K4A_WAIT_RESULT_SUCCEEDED)
                    {

                        k4a_wait_result_t queueCaptureResult = k4abt_tracker_enqueue_capture(tracker, sensorCapture, 0);
                        k4a_capture_release(sensorCapture);

                        popFrameResult = k4abt_tracker_pop_result(tracker, &bodyFrame, 0); // timeout_in_ms is set to 0
                        
                    }
                }
#pragma omp section
                {
                    k4a_wait_result_t getCaptureResult2 = k4a_device_get_capture(device2, &sensorCapture2, 0); // timeout_in_ms is set to 0
                    if (getCaptureResult2 == K4A_WAIT_RESULT_SUCCEEDED)
                    {
                        k4a_wait_result_t queueCaptureResult2 = k4abt_tracker_enqueue_capture(tracker2, sensorCapture2, 0);
                        k4a_capture_release(sensorCapture2);

                        popFrameResult2 = k4abt_tracker_pop_result(tracker2, &bodyFrame2, 0); // timeout_in_ms is set to 0
                        
                    }
                }
#pragma omp section
                {
                    k4a_wait_result_t getCaptureResult3 = k4a_device_get_capture(device3, &sensorCapture3, 0); // timeout_in_ms is set to 0
                    if (getCaptureResult3 == K4A_WAIT_RESULT_SUCCEEDED)
                    {
                        k4a_wait_result_t queueCaptureResult3 = k4abt_tracker_enqueue_capture(tracker3, sensorCapture3, 0);
                        k4a_capture_release(sensorCapture3);
                    
                        popFrameResult3 = k4abt_tracker_pop_result(tracker3, &bodyFrame3, 0); // timeout_in_ms is set to 0

                    }
                }
#pragma omp section
                {
                    k4a_wait_result_t getCaptureResult4 = k4a_device_get_capture(device4, &sensorCapture4, 0); // timeout_in_ms is set to 0
                    if (getCaptureResult4 == K4A_WAIT_RESULT_SUCCEEDED)
                    {
                        k4a_wait_result_t queueCaptureResult4 = k4abt_tracker_enqueue_capture(tracker4, sensorCapture4, 0);
                        k4a_capture_release(sensorCapture4);

                        popFrameResult4 = k4abt_tracker_pop_result(tracker4, &bodyFrame4, 0); // timeout_in_ms is set to 0

                    }
                }
#pragma omp section
                {
                    k4a_wait_result_t getCaptureResult5 = k4a_device_get_capture(device5, &sensorCapture5, 0); // timeout_in_ms is set to 0
                    if (getCaptureResult5 == K4A_WAIT_RESULT_SUCCEEDED)
                    {
                        k4a_wait_result_t queueCaptureResult5 = k4abt_tracker_enqueue_capture(tracker5, sensorCapture5, 0);
                        k4a_capture_release(sensorCapture5);

                        popFrameResult5 = k4abt_tracker_pop_result(tracker5, &bodyFrame5, 0); // timeout_in_ms is set to 0

                    }
                }
#pragma omp section
                {
                    k4a_wait_result_t getCaptureResult6 = k4a_device_get_capture(device6, &sensorCapture6, 0); // timeout_in_ms is set to 0
                    if (getCaptureResult6 == K4A_WAIT_RESULT_SUCCEEDED)
                    {
                        k4a_wait_result_t queueCaptureResult6 = k4abt_tracker_enqueue_capture(tracker6, sensorCapture6, 0);
                        k4a_capture_release(sensorCapture6);

                        popFrameResult6 = k4abt_tracker_pop_result(tracker6, &bodyFrame6, 0); // timeout_in_ms is set to 0

                    }
                }
#pragma omp section
                {
                    k4a_wait_result_t getCaptureResult7 = k4a_device_get_capture(device7, &sensorCapture7, 0); // timeout_in_ms is set to 0
                    if (getCaptureResult7 == K4A_WAIT_RESULT_SUCCEEDED)
                    {
                        k4a_wait_result_t queueCaptureResult7 = k4abt_tracker_enqueue_capture(tracker7, sensorCapture7, 0);
                        k4a_capture_release(sensorCapture7);

                        popFrameResult7 = k4abt_tracker_pop_result(tracker7, &bodyFrame7, 0); // timeout_in_ms is set to 0

                    }
                }
#pragma omp section
                {
                    k4a_wait_result_t getCaptureResult8 = k4a_device_get_capture(device8, &sensorCapture8, 0); // timeout_in_ms is set to 0
                    if (getCaptureResult8 == K4A_WAIT_RESULT_SUCCEEDED)
                    {
                        k4a_wait_result_t queueCaptureResult8 = k4abt_tracker_enqueue_capture(tracker8, sensorCapture8, 0);
                        k4a_capture_release(sensorCapture8);

                        popFrameResult8 = k4abt_tracker_pop_result(tracker8, &bodyFrame8, 0); // timeout_in_ms is set to 0

                    }
                }
            }
        }

        //Sleep(10);

        if (popFrameResult == K4A_WAIT_RESULT_SUCCEEDED &&
            popFrameResult2 == K4A_WAIT_RESULT_SUCCEEDED &&
            popFrameResult3 == K4A_WAIT_RESULT_SUCCEEDED &&
            popFrameResult4 == K4A_WAIT_RESULT_SUCCEEDED &&
            popFrameResult5 == K4A_WAIT_RESULT_SUCCEEDED &&
            popFrameResult6 == K4A_WAIT_RESULT_SUCCEEDED &&
            popFrameResult7 == K4A_WAIT_RESULT_SUCCEEDED &&
            popFrameResult8 == K4A_WAIT_RESULT_SUCCEEDED
            )
        {
            window3d.CleanJointsAndBones();
            if (iteration == 0) {
                update_is = 1;
            }
            iteration++;

            k4a_capture_t originalCapture = k4abt_frame_get_capture(bodyFrame);
            k4a_image_t depthImage = k4a_capture_get_depth_image(originalCapture);
            k4a_image_t colorImage = k4a_capture_get_color_image(originalCapture);
            window3d.UpdatePointClouds_UV(depthImage, colorImage, 0, extrinsic);


            window3d.m_joint.clear();
            window3d.m_joint2.clear();
            window3d.m_joint3.clear();
            window3d.m_joint4.clear();
            window3d.m_joint5.clear();
            window3d.m_joint6.clear();
            window3d.m_joint7.clear();
            window3d.m_joint8.clear();

            
                uint32_t numBodies = k4abt_frame_get_num_bodies(bodyFrame);



                for (uint32_t i = 0; i < numBodies; i++) 
                {
                    k4abt_body_t body;
                    VERIFY(k4abt_frame_get_body_skeleton(bodyFrame, i, &body.skeleton), "Get skeleton from body frame failed!");
                    body.id = k4abt_frame_get_body_id(bodyFrame, i);
            
                    // Assign the correct color based on the body id
                    Color color = g_bodyColors[body.id % g_bodyColors.size()];
                    color.a = 0.3f;
                    Color lowConfidenceColor = color;
                    lowConfidenceColor.a = 0;
                                      
                    // Visualize joints
                    for (int joint = 0; joint < static_cast<int>(K4ABT_JOINT_COUNT); joint++)
                    {
                        if (body.skeleton.joints[joint].confidence_level >= K4ABT_JOINT_CONFIDENCE_MEDIUM)
                        {
                            const k4a_float3_t& jointPosition = body.skeleton.joints[joint].position;
                            const k4a_quaternion_t& jointOrientation = body.skeleton.joints[joint].orientation;
                                                       
                            k4a_float3_t jointPosition_copy = body.skeleton.joints[joint].position;
                            k4a_float2_t point2d_xy;
                            int valid;
                            float f_x;
                            float f_y;

                            k4a_result_t convert_xy = k4a_calibration_3d_to_2d(&sensorCalibration,
                                &jointPosition_copy,
                                K4A_CALIBRATION_TYPE_DEPTH,
                                K4A_CALIBRATION_TYPE_DEPTH,
                                &point2d_xy,
                                &valid
                            );

                            if (K4A_RESULT_SUCCEEDED == convert_xy){
                                f_x = point2d_xy.xy.x;
                                f_y = point2d_xy.xy.y;

                                //printf("%f %f\n", f_x, f_y);
                            }                       

                            window3d.AddJoint(
                                jointPosition,
                                jointOrientation,
                                body.skeleton.joints[joint].confidence_level >= K4ABT_JOINT_CONFIDENCE_MEDIUM ? color : lowConfidenceColor,
                                0,
                                extrinsic
                            );

                            window3d.m_joint.push_back({joint,jointPosition.v[0] / 1000,jointPosition.v[1] / 1000,jointPosition.v[2] / 1000, f_x , f_y});
                        }
                    }
            
                    // Visualize bones
                    for (size_t boneIdx = 0; boneIdx < g_boneList.size(); boneIdx++)
                    {
                        k4abt_joint_id_t joint1 = g_boneList[boneIdx].first;
                        k4abt_joint_id_t joint2 = g_boneList[boneIdx].second;
            
                        if (body.skeleton.joints[joint1].confidence_level >= K4ABT_JOINT_CONFIDENCE_MEDIUM &&
                            body.skeleton.joints[joint2].confidence_level >= K4ABT_JOINT_CONFIDENCE_MEDIUM)
                        {
                       
                            bool confidentBone = body.skeleton.joints[joint1].confidence_level >= K4ABT_JOINT_CONFIDENCE_MEDIUM &&
                                body.skeleton.joints[joint2].confidence_level >= K4ABT_JOINT_CONFIDENCE_MEDIUM;
                            const k4a_float3_t& joint1Position = body.skeleton.joints[joint1].position;
                            const k4a_float3_t& joint2Position = body.skeleton.joints[joint2].position;
            
                            window3d.AddBone(joint1Position, joint2Position, confidentBone ? color : lowConfidenceColor, 0, extrinsic);
            
                        }
                    }
                }
            


            k4a_capture_release(originalCapture);
            //k4a_image_release(depthImage);
            //k4a_image_release(colorImage);
            k4abt_frame_release(bodyFrame);

            k4a_capture_t originalCapture2 = k4abt_frame_get_capture(bodyFrame2);
            k4a_image_t depthImage2 = k4a_capture_get_depth_image(originalCapture2);
            k4a_image_t colorImage2 = k4a_capture_get_color_image(originalCapture2);
            window3d.UpdatePointClouds_UV(depthImage2, colorImage2, 1, extrinsic);


            uint32_t numBodies2 = k4abt_frame_get_num_bodies(bodyFrame2);

            for (uint32_t i = 0; i < numBodies2; i++)
            {
                k4abt_body_t body;
                VERIFY(k4abt_frame_get_body_skeleton(bodyFrame2, i, &body.skeleton), "Get skeleton from body frame failed!");
                body.id = k4abt_frame_get_body_id(bodyFrame2, i);

                // Assign the correct color based on the body id
                Color color = g_bodyColors[body.id % g_bodyColors.size()];
                color.a = 0.3f;
                Color lowConfidenceColor = color;
                lowConfidenceColor.a = 0;

                // Visualize joints
                for (int joint = 0; joint < static_cast<int>(K4ABT_JOINT_COUNT); joint++)
                {
                    if (body.skeleton.joints[joint].confidence_level >= K4ABT_JOINT_CONFIDENCE_MEDIUM)
                    {
                        const k4a_float3_t& jointPosition = body.skeleton.joints[joint].position;
                        const k4a_quaternion_t& jointOrientation = body.skeleton.joints[joint].orientation;

                        k4a_float3_t jointPosition_copy = body.skeleton.joints[joint].position;
                        k4a_float2_t point2d_xy;
                        int valid;
                        float f_x;
                        float f_y;

                        k4a_result_t convert_xy = k4a_calibration_3d_to_2d(&sensorCalibration,
                            &jointPosition_copy,
                            K4A_CALIBRATION_TYPE_DEPTH,
                            K4A_CALIBRATION_TYPE_DEPTH,
                            &point2d_xy,
                            &valid
                        );

                        if (K4A_RESULT_SUCCEEDED == convert_xy) {
                            f_x = point2d_xy.xy.x;
                            f_y = point2d_xy.xy.y;

                            //printf("%f %f\n", f_x, f_y);
                        }

                        window3d.AddJoint(
                            jointPosition,
                            jointOrientation,
                            body.skeleton.joints[joint].confidence_level >= K4ABT_JOINT_CONFIDENCE_MEDIUM ? color : lowConfidenceColor,
                            1,
                            extrinsic
                        );

                        window3d.m_joint2.push_back({ joint,jointPosition.v[0] / 1000,jointPosition.v[1] / 1000,jointPosition.v[2] / 1000, f_x , f_y });
                    }
                }

                // Visualize bones
                for (size_t boneIdx = 0; boneIdx < g_boneList.size(); boneIdx++)
                {
                    k4abt_joint_id_t joint1 = g_boneList[boneIdx].first;
                    k4abt_joint_id_t joint2 = g_boneList[boneIdx].second;

                    if (body.skeleton.joints[joint1].confidence_level >= K4ABT_JOINT_CONFIDENCE_MEDIUM &&
                        body.skeleton.joints[joint2].confidence_level >= K4ABT_JOINT_CONFIDENCE_MEDIUM)
                    {


                        bool confidentBone = body.skeleton.joints[joint1].confidence_level >= K4ABT_JOINT_CONFIDENCE_MEDIUM &&
                            body.skeleton.joints[joint2].confidence_level >= K4ABT_JOINT_CONFIDENCE_MEDIUM;
                        const k4a_float3_t& joint1Position = body.skeleton.joints[joint1].position;
                        const k4a_float3_t& joint2Position = body.skeleton.joints[joint2].position;

                        window3d.AddBone(joint1Position, joint2Position, confidentBone ? color : lowConfidenceColor, 1, extrinsic);

                    }
                }
            }


            k4a_capture_release(originalCapture2);
            //k4a_image_release(depthImage2);
            //k4a_image_release(colorImage2);
            k4abt_frame_release(bodyFrame2);

            k4a_capture_t originalCapture3 = k4abt_frame_get_capture(bodyFrame3);
            k4a_image_t depthImage3 = k4a_capture_get_depth_image(originalCapture3);
            k4a_image_t colorImage3 = k4a_capture_get_color_image(originalCapture3);
            window3d.UpdatePointClouds_UV(depthImage3, colorImage3, 2, extrinsic);


            uint32_t numBodies3 = k4abt_frame_get_num_bodies(bodyFrame3);

            for (uint32_t i = 0; i < numBodies3; i++)
            {
                k4abt_body_t body;
                VERIFY(k4abt_frame_get_body_skeleton(bodyFrame3, i, &body.skeleton), "Get skeleton from body frame failed!");
                body.id = k4abt_frame_get_body_id(bodyFrame3, i);

                // Assign the correct color based on the body id
                Color color = g_bodyColors[body.id % g_bodyColors.size()];
                color.a = 0.3f;
                Color lowConfidenceColor = color;
                lowConfidenceColor.a = 0;

                // Visualize joints
                // Visualize joints
                for (int joint = 0; joint < static_cast<int>(K4ABT_JOINT_COUNT); joint++)
                {
                    if (body.skeleton.joints[joint].confidence_level >= K4ABT_JOINT_CONFIDENCE_MEDIUM)
                    {
                        const k4a_float3_t& jointPosition = body.skeleton.joints[joint].position;
                        const k4a_quaternion_t& jointOrientation = body.skeleton.joints[joint].orientation;

                        k4a_float3_t jointPosition_copy = body.skeleton.joints[joint].position;
                        k4a_float2_t point2d_xy;
                        int valid;
                        float f_x;
                        float f_y;

                        k4a_result_t convert_xy = k4a_calibration_3d_to_2d(&sensorCalibration,
                            &jointPosition_copy,
                            K4A_CALIBRATION_TYPE_DEPTH,
                            K4A_CALIBRATION_TYPE_DEPTH,
                            &point2d_xy,
                            &valid
                        );

                        if (K4A_RESULT_SUCCEEDED == convert_xy) {
                            f_x = point2d_xy.xy.x;
                            f_y = point2d_xy.xy.y;

                            //printf("%f %f\n", f_x, f_y);
                        }

                        window3d.AddJoint(
                            jointPosition,
                            jointOrientation,
                            body.skeleton.joints[joint].confidence_level >= K4ABT_JOINT_CONFIDENCE_MEDIUM ? color : lowConfidenceColor,
                            2,
                            extrinsic
                        );

                        window3d.m_joint3.push_back({ joint,jointPosition.v[0] / 1000,jointPosition.v[1] / 1000,jointPosition.v[2] / 1000, f_x , f_y });
                    }
                }

                // Visualize bones
                for (size_t boneIdx = 0; boneIdx < g_boneList.size(); boneIdx++)
                {
                    k4abt_joint_id_t joint1 = g_boneList[boneIdx].first;
                    k4abt_joint_id_t joint2 = g_boneList[boneIdx].second;

                    if (body.skeleton.joints[joint1].confidence_level >= K4ABT_JOINT_CONFIDENCE_MEDIUM &&
                        body.skeleton.joints[joint2].confidence_level >= K4ABT_JOINT_CONFIDENCE_MEDIUM)
                    {


                        bool confidentBone = body.skeleton.joints[joint1].confidence_level >= K4ABT_JOINT_CONFIDENCE_MEDIUM &&
                            body.skeleton.joints[joint2].confidence_level >= K4ABT_JOINT_CONFIDENCE_MEDIUM;
                        const k4a_float3_t& joint1Position = body.skeleton.joints[joint1].position;
                        const k4a_float3_t& joint2Position = body.skeleton.joints[joint2].position;

                        window3d.AddBone(joint1Position, joint2Position, confidentBone ? color : lowConfidenceColor, 2, extrinsic);

                    }
                }
            }

            k4a_capture_release(originalCapture3);
            //k4a_image_release(depthImage3);
            //k4a_image_release(colorImage3);
            k4abt_frame_release(bodyFrame3);

            k4a_capture_t originalCapture4 = k4abt_frame_get_capture(bodyFrame4);
            k4a_image_t depthImage4 = k4a_capture_get_depth_image(originalCapture4);
            k4a_image_t colorImage4 = k4a_capture_get_color_image(originalCapture4);
            window3d.UpdatePointClouds_UV(depthImage4, colorImage4, 3, extrinsic);


            uint32_t numBodies4 = k4abt_frame_get_num_bodies(bodyFrame4);

            for (uint32_t i = 0; i < numBodies4; i++)
            {
                k4abt_body_t body;
                VERIFY(k4abt_frame_get_body_skeleton(bodyFrame4, i, &body.skeleton), "Get skeleton from body frame failed!");
                body.id = k4abt_frame_get_body_id(bodyFrame4, i);

                // Assign the correct color based on the body id
                Color color = g_bodyColors[body.id % g_bodyColors.size()];
                color.a = 0.3f;
                Color lowConfidenceColor = color;
                lowConfidenceColor.a = 0;

                // Visualize joints
                // Visualize joints
                for (int joint = 0; joint < static_cast<int>(K4ABT_JOINT_COUNT); joint++)
                {
                    if (body.skeleton.joints[joint].confidence_level >= K4ABT_JOINT_CONFIDENCE_MEDIUM)
                    {
                        const k4a_float3_t& jointPosition = body.skeleton.joints[joint].position;
                        const k4a_quaternion_t& jointOrientation = body.skeleton.joints[joint].orientation;

                        k4a_float3_t jointPosition_copy = body.skeleton.joints[joint].position;
                        k4a_float2_t point2d_xy;
                        int valid;
                        float f_x;
                        float f_y;

                        k4a_result_t convert_xy = k4a_calibration_3d_to_2d(&sensorCalibration,
                            &jointPosition_copy,
                            K4A_CALIBRATION_TYPE_DEPTH,
                            K4A_CALIBRATION_TYPE_DEPTH,
                            &point2d_xy,
                            &valid
                        );

                        if (K4A_RESULT_SUCCEEDED == convert_xy) {
                            f_x = point2d_xy.xy.x;
                            f_y = point2d_xy.xy.y;

                            //printf("%f %f\n", f_x, f_y);
                        }

                        window3d.AddJoint(
                            jointPosition,
                            jointOrientation,
                            body.skeleton.joints[joint].confidence_level >= K4ABT_JOINT_CONFIDENCE_MEDIUM ? color : lowConfidenceColor,
                            3,
                            extrinsic
                        );

                        window3d.m_joint4.push_back({ joint,jointPosition.v[0] / 1000,jointPosition.v[1] / 1000,jointPosition.v[2] / 1000, f_x , f_y });
                    }
                }

                // Visualize bones
                for (size_t boneIdx = 0; boneIdx < g_boneList.size(); boneIdx++)
                {
                    k4abt_joint_id_t joint1 = g_boneList[boneIdx].first;
                    k4abt_joint_id_t joint2 = g_boneList[boneIdx].second;

                    if (body.skeleton.joints[joint1].confidence_level >= K4ABT_JOINT_CONFIDENCE_MEDIUM &&
                        body.skeleton.joints[joint2].confidence_level >= K4ABT_JOINT_CONFIDENCE_MEDIUM)
                    {


                        bool confidentBone = body.skeleton.joints[joint1].confidence_level >= K4ABT_JOINT_CONFIDENCE_MEDIUM &&
                            body.skeleton.joints[joint2].confidence_level >= K4ABT_JOINT_CONFIDENCE_MEDIUM;
                        const k4a_float3_t& joint1Position = body.skeleton.joints[joint1].position;
                        const k4a_float3_t& joint2Position = body.skeleton.joints[joint2].position;

                        window3d.AddBone(joint1Position, joint2Position, confidentBone ? color : lowConfidenceColor, 3, extrinsic);

                    }
                }
            }

            k4a_capture_release(originalCapture4);
            //k4a_image_release(depthImage4);
            //k4a_image_release(colorImage4);
            k4abt_frame_release(bodyFrame4);

            k4a_capture_t originalCapture5 = k4abt_frame_get_capture(bodyFrame5);
            k4a_image_t depthImage5 = k4a_capture_get_depth_image(originalCapture5);
            k4a_image_t colorImage5 = k4a_capture_get_color_image(originalCapture5);
            window3d.UpdatePointClouds_UV(depthImage5, colorImage5, 4, extrinsic);


            uint32_t numBodies5 = k4abt_frame_get_num_bodies(bodyFrame5);

            for (uint32_t i = 0; i < numBodies5; i++)
            {
                k4abt_body_t body;
                VERIFY(k4abt_frame_get_body_skeleton(bodyFrame5, i, &body.skeleton), "Get skeleton from body frame failed!");
                body.id = k4abt_frame_get_body_id(bodyFrame5, i);

                // Assign the correct color based on the body id
                Color color = g_bodyColors[body.id % g_bodyColors.size()];
                color.a = 0.3f;
                Color lowConfidenceColor = color;
                lowConfidenceColor.a = 0;


                // Visualize joints
                for (int joint = 0; joint < static_cast<int>(K4ABT_JOINT_COUNT); joint++)
                {
                    if (body.skeleton.joints[joint].confidence_level >= K4ABT_JOINT_CONFIDENCE_MEDIUM)
                    {
                        const k4a_float3_t& jointPosition = body.skeleton.joints[joint].position;
                        const k4a_quaternion_t& jointOrientation = body.skeleton.joints[joint].orientation;

                        k4a_float3_t jointPosition_copy = body.skeleton.joints[joint].position;
                        k4a_float2_t point2d_xy;
                        int valid;
                        float f_x;
                        float f_y;

                        k4a_result_t convert_xy = k4a_calibration_3d_to_2d(&sensorCalibration,
                            &jointPosition_copy,
                            K4A_CALIBRATION_TYPE_DEPTH,
                            K4A_CALIBRATION_TYPE_DEPTH,
                            &point2d_xy,
                            &valid
                        );

                        if (K4A_RESULT_SUCCEEDED == convert_xy) {
                            f_x = point2d_xy.xy.x;
                            f_y = point2d_xy.xy.y;

                            //printf("%f %f\n", f_x, f_y);
                        }

                        window3d.AddJoint(
                            jointPosition,
                            jointOrientation,
                            body.skeleton.joints[joint].confidence_level >= K4ABT_JOINT_CONFIDENCE_MEDIUM ? color : lowConfidenceColor,
                            4,
                            extrinsic
                        );

                        window3d.m_joint5.push_back({ joint,jointPosition.v[0] / 1000,jointPosition.v[1] / 1000,jointPosition.v[2] / 1000, f_x , f_y });
                    }
                }

                // Visualize bones
                for (size_t boneIdx = 0; boneIdx < g_boneList.size(); boneIdx++)
                {
                    k4abt_joint_id_t joint1 = g_boneList[boneIdx].first;
                    k4abt_joint_id_t joint2 = g_boneList[boneIdx].second;

                    if (body.skeleton.joints[joint1].confidence_level >= K4ABT_JOINT_CONFIDENCE_MEDIUM &&
                        body.skeleton.joints[joint2].confidence_level >= K4ABT_JOINT_CONFIDENCE_MEDIUM)
                    {


                        bool confidentBone = body.skeleton.joints[joint1].confidence_level >= K4ABT_JOINT_CONFIDENCE_MEDIUM &&
                            body.skeleton.joints[joint2].confidence_level >= K4ABT_JOINT_CONFIDENCE_MEDIUM;
                        const k4a_float3_t& joint1Position = body.skeleton.joints[joint1].position;
                        const k4a_float3_t& joint2Position = body.skeleton.joints[joint2].position;

                        window3d.AddBone(joint1Position, joint2Position, confidentBone ? color : lowConfidenceColor, 4, extrinsic);

                    }
                }
            }



            k4a_capture_release(originalCapture5);
            //k4a_image_release(depthImage5);
            //k4a_image_release(colorImage5);
            k4abt_frame_release(bodyFrame5);

            k4a_capture_t originalCapture6 = k4abt_frame_get_capture(bodyFrame6);
            k4a_image_t depthImage6 = k4a_capture_get_depth_image(originalCapture6);
            k4a_image_t colorImage6 = k4a_capture_get_color_image(originalCapture6);
            window3d.UpdatePointClouds_UV(depthImage6, colorImage6, 5, extrinsic);

            uint32_t numBodies6 = k4abt_frame_get_num_bodies(bodyFrame6);

            for (uint32_t i = 0; i < numBodies6; i++)
            {
                k4abt_body_t body;
                VERIFY(k4abt_frame_get_body_skeleton(bodyFrame6, i, &body.skeleton), "Get skeleton from body frame failed!");
                body.id = k4abt_frame_get_body_id(bodyFrame6, i);

                // Assign the correct color based on the body id
                Color color = g_bodyColors[body.id % g_bodyColors.size()];
                color.a = 0.3f;
                Color lowConfidenceColor = color;
                lowConfidenceColor.a = 0;

                // Visualize joints
                for (int joint = 0; joint < static_cast<int>(K4ABT_JOINT_COUNT); joint++)
                {
                    if (body.skeleton.joints[joint].confidence_level >= K4ABT_JOINT_CONFIDENCE_MEDIUM)
                    {
                        const k4a_float3_t& jointPosition = body.skeleton.joints[joint].position;
                        const k4a_quaternion_t& jointOrientation = body.skeleton.joints[joint].orientation;

                        k4a_float3_t jointPosition_copy = body.skeleton.joints[joint].position;
                        k4a_float2_t point2d_xy;
                        int valid;
                        float f_x;
                        float f_y;

                        k4a_result_t convert_xy = k4a_calibration_3d_to_2d(&sensorCalibration,
                            &jointPosition_copy,
                            K4A_CALIBRATION_TYPE_DEPTH,
                            K4A_CALIBRATION_TYPE_DEPTH,
                            &point2d_xy,
                            &valid
                        );

                        if (K4A_RESULT_SUCCEEDED == convert_xy) {
                            f_x = point2d_xy.xy.x;
                            f_y = point2d_xy.xy.y;

                            //printf("%f %f\n", f_x, f_y);
                        }

                        window3d.AddJoint(
                            jointPosition,
                            jointOrientation,
                            body.skeleton.joints[joint].confidence_level >= K4ABT_JOINT_CONFIDENCE_MEDIUM ? color : lowConfidenceColor,
                            5,
                            extrinsic
                        );

                        window3d.m_joint6.push_back({ joint,jointPosition.v[0] / 1000,jointPosition.v[1] / 1000,jointPosition.v[2] / 1000, f_x , f_y });
                    }
                }

                // Visualize bones
                for (size_t boneIdx = 0; boneIdx < g_boneList.size(); boneIdx++)
                {
                    k4abt_joint_id_t joint1 = g_boneList[boneIdx].first;
                    k4abt_joint_id_t joint2 = g_boneList[boneIdx].second;

                    if (body.skeleton.joints[joint1].confidence_level >= K4ABT_JOINT_CONFIDENCE_MEDIUM &&
                        body.skeleton.joints[joint2].confidence_level >= K4ABT_JOINT_CONFIDENCE_MEDIUM)
                    {


                        bool confidentBone = body.skeleton.joints[joint1].confidence_level >= K4ABT_JOINT_CONFIDENCE_MEDIUM &&
                            body.skeleton.joints[joint2].confidence_level >= K4ABT_JOINT_CONFIDENCE_MEDIUM;
                        const k4a_float3_t& joint1Position = body.skeleton.joints[joint1].position;
                        const k4a_float3_t& joint2Position = body.skeleton.joints[joint2].position;

                        window3d.AddBone(joint1Position, joint2Position, confidentBone ? color : lowConfidenceColor, 5, extrinsic);

                    }
                }
            }

            k4a_capture_release(originalCapture6);
            //k4a_image_release(depthImage6);
            //k4a_image_release(colorImage6);
            k4abt_frame_release(bodyFrame6);


            k4a_capture_t originalCapture7 = k4abt_frame_get_capture(bodyFrame7);
            k4a_image_t depthImage7 = k4a_capture_get_depth_image(originalCapture7);
            k4a_image_t colorImage7 = k4a_capture_get_color_image(originalCapture7);
            window3d.UpdatePointClouds_UV(depthImage7, colorImage7, 6, extrinsic);


            uint32_t numBodies7 = k4abt_frame_get_num_bodies(bodyFrame7);

            for (uint32_t i = 0; i < numBodies7; i++)
            {
                k4abt_body_t body;
                VERIFY(k4abt_frame_get_body_skeleton(bodyFrame7, i, &body.skeleton), "Get skeleton from body frame failed!");
                body.id = k4abt_frame_get_body_id(bodyFrame7, i);

                // Assign the correct color based on the body id
                Color color = g_bodyColors[body.id % g_bodyColors.size()];
                color.a = 0.3f;
                Color lowConfidenceColor = color;
                lowConfidenceColor.a = 0;

                // Visualize joints
                for (int joint = 0; joint < static_cast<int>(K4ABT_JOINT_COUNT); joint++)
                {
                    if (body.skeleton.joints[joint].confidence_level >= K4ABT_JOINT_CONFIDENCE_MEDIUM)
                    {
                        const k4a_float3_t& jointPosition = body.skeleton.joints[joint].position;
                        const k4a_quaternion_t& jointOrientation = body.skeleton.joints[joint].orientation;

                        k4a_float3_t jointPosition_copy = body.skeleton.joints[joint].position;
                        k4a_float2_t point2d_xy;
                        int valid;
                        float f_x;
                        float f_y;

                        k4a_result_t convert_xy = k4a_calibration_3d_to_2d(&sensorCalibration,
                            &jointPosition_copy,
                            K4A_CALIBRATION_TYPE_DEPTH,
                            K4A_CALIBRATION_TYPE_DEPTH,
                            &point2d_xy,
                            &valid
                        );

                        if (K4A_RESULT_SUCCEEDED == convert_xy) {
                            f_x = point2d_xy.xy.x;
                            f_y = point2d_xy.xy.y;

                            //printf("%f %f\n", f_x, f_y);
                        }

                        window3d.AddJoint(
                            jointPosition,
                            jointOrientation,
                            body.skeleton.joints[joint].confidence_level >= K4ABT_JOINT_CONFIDENCE_MEDIUM ? color : lowConfidenceColor,
                            6,
                            extrinsic
                        );

                        window3d.m_joint7.push_back({ joint,jointPosition.v[0] / 1000,jointPosition.v[1] / 1000,jointPosition.v[2] / 1000, f_x , f_y });
                    }
                }

                // Visualize bones
                for (size_t boneIdx = 0; boneIdx < g_boneList.size(); boneIdx++)
                {
                    k4abt_joint_id_t joint1 = g_boneList[boneIdx].first;
                    k4abt_joint_id_t joint2 = g_boneList[boneIdx].second;

                    if (body.skeleton.joints[joint1].confidence_level >= K4ABT_JOINT_CONFIDENCE_MEDIUM &&
                        body.skeleton.joints[joint2].confidence_level >= K4ABT_JOINT_CONFIDENCE_MEDIUM)
                    {


                        bool confidentBone = body.skeleton.joints[joint1].confidence_level >= K4ABT_JOINT_CONFIDENCE_MEDIUM &&
                            body.skeleton.joints[joint2].confidence_level >= K4ABT_JOINT_CONFIDENCE_MEDIUM;
                        const k4a_float3_t& joint1Position = body.skeleton.joints[joint1].position;
                        const k4a_float3_t& joint2Position = body.skeleton.joints[joint2].position;

                        window3d.AddBone(joint1Position, joint2Position, confidentBone ? color : lowConfidenceColor, 6, extrinsic);

                    }
                }
            }


            k4a_capture_release(originalCapture7);
            //k4a_image_release(depthImage7);
            //k4a_image_release(colorImage7);
            k4abt_frame_release(bodyFrame7);
           

            k4a_capture_t originalCapture8 = k4abt_frame_get_capture(bodyFrame8);
            k4a_image_t depthImage8 = k4a_capture_get_depth_image(originalCapture8);
            k4a_image_t colorImage8 = k4a_capture_get_color_image(originalCapture8);
            window3d.UpdatePointClouds_UV(depthImage8, colorImage8, 7, extrinsic);


            uint32_t numBodies8 = k4abt_frame_get_num_bodies(bodyFrame8);

            for (uint32_t i = 0; i < numBodies8; i++)
            {
                k4abt_body_t body;
                VERIFY(k4abt_frame_get_body_skeleton(bodyFrame8, i, &body.skeleton), "Get skeleton from body frame failed!");
                body.id = k4abt_frame_get_body_id(bodyFrame8, i);

                // Assign the correct color based on the body id
                Color color = g_bodyColors[body.id % g_bodyColors.size()];
                color.a = 0.3f;
                Color lowConfidenceColor = color;
                lowConfidenceColor.a = 0;

                // Visualize joints
                for (int joint = 0; joint < static_cast<int>(K4ABT_JOINT_COUNT); joint++)
                {
                    if (body.skeleton.joints[joint].confidence_level >= K4ABT_JOINT_CONFIDENCE_MEDIUM)
                    {
                        const k4a_float3_t& jointPosition = body.skeleton.joints[joint].position;
                        const k4a_quaternion_t& jointOrientation = body.skeleton.joints[joint].orientation;

                        k4a_float3_t jointPosition_copy = body.skeleton.joints[joint].position;
                        k4a_float2_t point2d_xy;
                        int valid;
                        float f_x;
                        float f_y;

                        k4a_result_t convert_xy = k4a_calibration_3d_to_2d(&sensorCalibration,
                            &jointPosition_copy,
                            K4A_CALIBRATION_TYPE_DEPTH,
                            K4A_CALIBRATION_TYPE_DEPTH,
                            &point2d_xy,
                            &valid
                        );

                        if (K4A_RESULT_SUCCEEDED == convert_xy) {
                            f_x = point2d_xy.xy.x;
                            f_y = point2d_xy.xy.y;

                            //printf("%f %f\n", f_x, f_y);
                        }

                        window3d.AddJoint(
                            jointPosition,
                            jointOrientation,
                            body.skeleton.joints[joint].confidence_level >= K4ABT_JOINT_CONFIDENCE_MEDIUM ? color : lowConfidenceColor,
                            7,
                            extrinsic
                        );

                        window3d.m_joint8.push_back({ joint,jointPosition.v[0] / 1000,jointPosition.v[1] / 1000,jointPosition.v[2] / 1000, f_x , f_y });
                    }
                }

                // Visualize bones
                for (size_t boneIdx = 0; boneIdx < g_boneList.size(); boneIdx++)
                {
                    k4abt_joint_id_t joint1 = g_boneList[boneIdx].first;
                    k4abt_joint_id_t joint2 = g_boneList[boneIdx].second;

                    if (body.skeleton.joints[joint1].confidence_level >= K4ABT_JOINT_CONFIDENCE_MEDIUM &&
                        body.skeleton.joints[joint2].confidence_level >= K4ABT_JOINT_CONFIDENCE_MEDIUM)
                    {


                        bool confidentBone = body.skeleton.joints[joint1].confidence_level >= K4ABT_JOINT_CONFIDENCE_MEDIUM &&
                            body.skeleton.joints[joint2].confidence_level >= K4ABT_JOINT_CONFIDENCE_MEDIUM;
                        const k4a_float3_t& joint1Position = body.skeleton.joints[joint1].position;
                        const k4a_float3_t& joint2Position = body.skeleton.joints[joint2].position;

                        window3d.AddBone(joint1Position, joint2Position, confidentBone ? color : lowConfidenceColor, 7, extrinsic);

                    }
                }
            }

            k4a_capture_release(originalCapture8);
            //k4a_image_release(depthImage8);
            //k4a_image_release(colorImage8);
            k4abt_frame_release(bodyFrame8);

            int joint_count = window3d.m_joint.size();
            int joint_count2 = window3d.m_joint2.size();
            int joint_count3 = window3d.m_joint3.size();
            int joint_count4 = window3d.m_joint4.size();
            int joint_count5 = window3d.m_joint5.size();
            int joint_count6 = window3d.m_joint6.size();
            int joint_count7 = window3d.m_joint7.size();
            int joint_count8 = window3d.m_joint8.size();


            double tetha_cam1 = 100;
            double tetha_cam2 = 100;
            double tetha_cam3 = 100;
            double tetha_cam4 = 100;
            double tetha_cam5 = 100;
            double tetha_cam6 = 100;
            double tetha_cam7 = 100;
            double tetha_cam8 = 100;


            int add_joint_cam1 = 0;
            int add_joint_cam2 = 0;
            int add_joint_cam3 = 0;
            int add_joint_cam4 = 0;
            int add_joint_cam5 = 0;
            int add_joint_cam6 = 0;
            int add_joint_cam7 = 0;
            int add_joint_cam8 = 0;

            
            printf("cam1_joint_size %d\n", joint_count);
            printf("cam2_joint_size %d\n", joint_count2);
            printf("cam3_joint_size %d\n", joint_count3);
            printf("cam4_joint_size %d\n", joint_count4);
            printf("cam5_joint_size %d\n", joint_count5);
            printf("cam6_joint_size %d\n", joint_count6);
            printf("cam7_joint_size %d\n", joint_count7);
            printf("cam8_joint_size %d\n", joint_count8);

            uint8_t* buffer = k4a_image_get_buffer(depthImage);
            cv::Mat depth_map = cv::Mat(576, 640, CV_16UC1, (void*)buffer, cv::Mat::AUTO_STEP);
            cv::Mat depth_map_clone = depth_map.clone();
           
            uint8_t* buffer2 = k4a_image_get_buffer(depthImage2);
            cv::Mat depth_map2 = cv::Mat(576, 640, CV_16UC1, (void*)buffer2, cv::Mat::AUTO_STEP);
            cv::Mat depth_map_clone2 = depth_map2.clone();

            uint8_t* buffer3 = k4a_image_get_buffer(depthImage3);
            cv::Mat depth_map3 = cv::Mat(576, 640, CV_16UC1, (void*)buffer3, cv::Mat::AUTO_STEP);
            cv::Mat depth_map_clone3 = depth_map3.clone();

            uint8_t* buffer4 = k4a_image_get_buffer(depthImage4);
            cv::Mat depth_map4 = cv::Mat(576, 640, CV_16UC1, (void*)buffer4, cv::Mat::AUTO_STEP);
            cv::Mat depth_map_clone4 = depth_map4.clone();

            uint8_t* buffer5 = k4a_image_get_buffer(depthImage5);
            cv::Mat depth_map5 = cv::Mat(576, 640, CV_16UC1, (void*)buffer5, cv::Mat::AUTO_STEP);
            cv::Mat depth_map_clone5 = depth_map5.clone();

            uint8_t* buffer6 = k4a_image_get_buffer(depthImage6);
            cv::Mat depth_map6 = cv::Mat(576, 640, CV_16UC1, (void*)buffer6, cv::Mat::AUTO_STEP);
            cv::Mat depth_map_clone6 = depth_map6.clone();

            uint8_t* buffer7 = k4a_image_get_buffer(depthImage7);
            cv::Mat depth_map7 = cv::Mat(576, 640, CV_16UC1, (void*)buffer7, cv::Mat::AUTO_STEP);
            cv::Mat depth_map_clone7 = depth_map7.clone();

            uint8_t* buffer8 = k4a_image_get_buffer(depthImage8);
            cv::Mat depth_map8 = cv::Mat(576, 640, CV_16UC1, (void*)buffer8, cv::Mat::AUTO_STEP);
            cv::Mat depth_map_clone8 = depth_map8.clone();


            uint8_t* buffer_c = k4a_image_get_buffer(colorImage);
            cv::Mat color_map = cv::Mat(720, 1280, CV_8UC4, (void*)buffer_c, cv::Mat::AUTO_STEP);
            cv::Mat color_map_clone = color_map.clone();

            uint8_t* buffer_c2 = k4a_image_get_buffer(colorImage2);
            cv::Mat color_map2 = cv::Mat(720, 1280, CV_8UC4, (void*)buffer_c2, cv::Mat::AUTO_STEP);
            cv::Mat color_map_clone2 = color_map2.clone();

            uint8_t* buffer_c3 = k4a_image_get_buffer(colorImage3);
            cv::Mat color_map3 = cv::Mat(720, 1280, CV_8UC4, (void*)buffer_c3, cv::Mat::AUTO_STEP);
            cv::Mat color_map_clone3 = color_map3.clone();

            uint8_t* buffer_c4 = k4a_image_get_buffer(colorImage4);
            cv::Mat color_map4 = cv::Mat(720, 1280, CV_8UC4, (void*)buffer_c4, cv::Mat::AUTO_STEP);
            cv::Mat color_map_clone4 = color_map4.clone();

            uint8_t* buffer_c5 = k4a_image_get_buffer(colorImage5);
            cv::Mat color_map5 = cv::Mat(720, 1280, CV_8UC4, (void*)buffer_c5, cv::Mat::AUTO_STEP);
            cv::Mat color_map_clone5 = color_map5.clone();

            uint8_t* buffer_c6 = k4a_image_get_buffer(colorImage6);
            cv::Mat color_map6 = cv::Mat(720, 1280, CV_8UC4, (void*)buffer_c6, cv::Mat::AUTO_STEP);
            cv::Mat color_map_clone6 = color_map6.clone();

            uint8_t* buffer_c7 = k4a_image_get_buffer(colorImage7);
            cv::Mat color_map7 = cv::Mat(720, 1280, CV_8UC4, (void*)buffer_c7, cv::Mat::AUTO_STEP);
            cv::Mat color_map_clone7 = color_map7.clone();

            uint8_t* buffer_c8 = k4a_image_get_buffer(colorImage8);
            cv::Mat color_map8 = cv::Mat(720, 1280, CV_8UC4, (void*)buffer_c8, cv::Mat::AUTO_STEP);
            cv::Mat color_map_clone8 = color_map8.clone();


                        

            cv::normalize(depth_map_clone, depth_map_clone, 0, 255, cv::NORM_MINMAX, CV_8UC1);
            cv::normalize(depth_map_clone2, depth_map_clone2, 0, 255, cv::NORM_MINMAX, CV_8UC1);
            cv::normalize(depth_map_clone3, depth_map_clone3, 0, 255, cv::NORM_MINMAX, CV_8UC1);
            cv::normalize(depth_map_clone4, depth_map_clone4, 0, 255, cv::NORM_MINMAX, CV_8UC1);
            cv::normalize(depth_map_clone5, depth_map_clone5, 0, 255, cv::NORM_MINMAX, CV_8UC1);
            cv::normalize(depth_map_clone6, depth_map_clone6, 0, 255, cv::NORM_MINMAX, CV_8UC1);
            cv::normalize(depth_map_clone7, depth_map_clone7, 0, 255, cv::NORM_MINMAX, CV_8UC1);
            cv::normalize(depth_map_clone8, depth_map_clone8, 0, 255, cv::NORM_MINMAX, CV_8UC1);

            //depth_map.convertTo(depth_map, cv::COLOR_GRAY2RGB);




            if(only_image == 1){
                cv::imwrite("output/" + to_string(iteration) + "_cam2_depth" + ".bmp", depth_map_clone2);
                cv::imwrite("output/" + to_string(iteration) + "_cam3_depth" + ".bmp", depth_map_clone3);
                cv::imwrite("output/" + to_string(iteration) + "_cam4_depth" + ".bmp", depth_map_clone4);
                cv::imwrite("output/" + to_string(iteration) + "_cam5_depth" + ".bmp", depth_map_clone5);
                cv::imwrite("output/" + to_string(iteration) + "_cam1_depth" + ".bmp", depth_map_clone);
                cv::imwrite("output/" + to_string(iteration) + "_cam6_depth" + ".bmp", depth_map_clone6);
                cv::imwrite("output/" + to_string(iteration) + "_cam7_depth" + ".bmp", depth_map_clone7);
                cv::imwrite("output/" + to_string(iteration) + "_cam8_depth" + ".bmp", depth_map_clone8);
                             
                cv::imwrite("output/" + to_string(iteration) + "_cam1_color" + ".bmp", color_map_clone);
                cv::imwrite("output/" + to_string(iteration) + "_cam2_color" + ".bmp", color_map_clone2);
                cv::imwrite("output/" + to_string(iteration) + "_cam3_color" + ".bmp", color_map_clone3);
                cv::imwrite("output/" + to_string(iteration) + "_cam4_color" + ".bmp", color_map_clone4);
                cv::imwrite("output/" + to_string(iteration) + "_cam5_color" + ".bmp", color_map_clone5);
                cv::imwrite("output/" + to_string(iteration) + "_cam6_color" + ".bmp", color_map_clone6);
                cv::imwrite("output/" + to_string(iteration) + "_cam7_color" + ".bmp", color_map_clone7);
                cv::imwrite("output/" + to_string(iteration) + "_cam8_color" + ".bmp", color_map_clone8);

                joint_count = 0;
                joint_count2 = 0;
                joint_count3 = 0;
                joint_count4 = 0;
                joint_count5 = 0;
                joint_count6 = 0;
                joint_count7 = 0;
                joint_count8 = 0;
            }


                      
            printf("------------------------- cam1 -----------------------------\n");

            int joint_0_x = 0;
            int joint_0_y = 0;
            int joint_6_x = 0;
            int joint_6_y = 0;
            int joint_12_x = 0;
            int joint_12_y = 0;
            int joint_13_x = 0;
            int joint_13_y = 0;
            int joint_5_x = 0;
            int joint_5_y = 0;
            int joint_18_x = 0;
            int joint_18_y = 0;
            int joint_22_x = 0;
            int joint_22_y = 0;

            for (int i = 0; i < joint_count; i++) {
                int joint_x = int(floor((window3d.m_joint[i].int_x) + 0.5));
                int joint_y = int(floor((window3d.m_joint[i].int_y) + 0.5));

                if (joint_x < 0) {
                    continue;
                }

                if (joint_y < 0) {
                    continue;
                }

                float zz = (float)depth_map.at<ushort>(joint_y, joint_x) / 1000;

                //표면의 z값이 0인경우 제거
                if (zz != 0) {
                    //표면과 보다 조인트의 거리가 카메라와 가까운 경우  제거
                    if (window3d.m_joint[i].z > zz) {
                        //표면과 조인트의 거리가 15cm가 넘는 경우 제거
                        if (window3d.m_joint[i].z - zz < 0.15) {                            
                            //뎁스 해상도를 넘어가는 조인트 제거
                            if (joint_y > 10) {
                                int joint_num = window3d.m_joint[i].num;
                                // 손 발 제거 
                                if (joint_num != 8 && joint_num != 9 && joint_num != 10 && joint_num != 15 && joint_num != 16 && joint_num != 17 && joint_num != 28 && joint_num != 29 && joint_num != 30 && joint_num != 31 && joint_num != 32) {
                                    printf("add joint %d : x = %d, y = %d, z = %f, depthmap = %f\n", window3d.m_joint[i].num, joint_x, joint_y, window3d.m_joint[i].z, zz);

                                    if (window3d.m_joint[i].num == 0) {
                                        joint_0_x = joint_x;
                                        joint_0_y = joint_y;
                                    }
                                    if (window3d.m_joint[i].num == 18) {
                                        joint_18_x = joint_x;
                                        joint_18_y = joint_y;
                                    }
                                    if (window3d.m_joint[i].num == 22) {
                                        joint_22_x = joint_x;
                                        joint_22_y = joint_y;
                                    }
                                    if (window3d.m_joint[i].num == 12) {
                                        joint_12_x = joint_x;
                                        joint_12_y = joint_y;
                                    }
                                    if (window3d.m_joint[i].num == 5) {
                                        joint_5_x = joint_x;
                                        joint_5_y = joint_y;
                                    }
                                    if (window3d.m_joint[i].num == 6) {
                                        joint_6_x = joint_x;
                                        joint_6_y = joint_y;
                                    }
                                    if (window3d.m_joint[i].num == 13) {
                                        joint_13_x = joint_x;
                                        joint_13_y = joint_y;
                                    }

                                    add_joint_cam1++;

                                    //cv::Point p;
                                    //p.x = joint_x + 5;
                                    //p.y = joint_y + 5;
                                    //
                                    //cv::circle(depth_map_clone, cv::Point(joint_x, joint_y), 2, cv::Scalar(255, 255, 255), -1, 8, 0);
                                    //cv::putText(depth_map_clone, to_string(window3d.m_joint[i].num), p, 2, 0.4, cv::Scalar(255, 0, 0), 1, 1);

                                    window3d.m_joint_result.push_back({ window3d.m_joint[i].num,window3d.m_joint[i].x ,window3d.m_joint[i].y ,window3d.m_joint[i].z});
                                }
                                else {
                                    printf("remove joint %d : x = %d, y = %d, z = %f, depthmap = %f\n", window3d.m_joint[i].num, joint_x, joint_y, window3d.m_joint[i].z, zz);
                                }
                            }else {
                                printf("remove joint %d : x = %d, y = %d, z = %f, depthmap = %f\n", window3d.m_joint[i].num, joint_x, joint_y, window3d.m_joint[i].z, zz);
                            }
                        }else {
                            printf("remove joint %d : x = %d, y = %d, z = %f, depthmap = %f\n", window3d.m_joint[i].num, joint_x, joint_y, window3d.m_joint[i].z, zz);
                        }
                    }else {
                        printf("remove joint %d : x = %d, y = %d, z = %f, depthmap = %f\n", window3d.m_joint[i].num, joint_x, joint_y, window3d.m_joint[i].z, zz);
                    }
                }else {
                    printf("remove joint %d : x = %d, y = %d, z = %f, depthmap = %f\n", window3d.m_joint[i].num, joint_x, joint_y, window3d.m_joint[i].z, zz);
                }
            }

            if (joint_18_x != 0 && joint_18_y != 0 && joint_22_x != 0 && joint_22_y != 0 && joint_12_x != 0 && joint_12_y != 0 && joint_5_x != 0 && joint_5_y != 0) {
                               
                //line(depth_map_clone, cv::Point(joint_18_x, joint_18_y), cv::Point(joint_22_x, joint_22_y), cv::Scalar::all(255), 1, 8, 0);
                //line(depth_map_clone, cv::Point(joint_12_x, joint_12_y), cv::Point(joint_5_x, joint_5_y), cv::Scalar::all(255), 1, 8, 0);
              
                cv::Point p;              
                cv::Point p2;
                p.x = joint_22_x + 15;
                p.y = joint_22_y - 15;
                double dy = (double)joint_22_y - (double)joint_18_y;
                double dx = (double)joint_22_x - (double)joint_18_x;
               
                p2.x = joint_5_x + 15;
                p2.y = joint_5_y - 15;
                double dy2 = (double)joint_5_y - (double)joint_12_y;
                double dx2 = (double)joint_5_x - (double)joint_12_x;
                
                double tehta = atan(dy / dx);
                double tehta2 = atan(dy2 / dx2);
               
                //cv::putText(depth_map_clone, to_string(tehta), p, 2, 0.4, cv::Scalar(255, 255, 255), 1, 1);
                //cv::putText(depth_map_clone, to_string(tehta2), p2, 2, 0.4, cv::Scalar(255, 255, 255), 1, 1);
               
                tetha_cam1 = std::fabs(tehta);
            
            }
     
            printf("------------------------- cam2 -----------------------------\n");

            joint_0_x = 0;
            joint_0_y = 0;
            joint_6_x = 0;
            joint_6_y = 0;
            joint_12_x = 0;
            joint_12_y = 0;
            joint_13_x = 0;
            joint_13_y = 0;
            joint_5_x = 0;
            joint_5_y = 0;
            joint_18_x = 0;
            joint_18_y = 0;
            joint_22_x = 0;
            joint_22_y = 0;

            for (int i = 0; i < joint_count2; i++) {
                int joint_x = int(floor((window3d.m_joint2[i].int_x) + 0.5));
                int joint_y = int(floor((window3d.m_joint2[i].int_y) + 0.5));

                if (joint_x < 0) {
                    continue;
                }

                if (joint_y < 0) {
                    continue;
                }

                float zz = (float)depth_map2.at<ushort>(joint_y, joint_x) / 1000;

                //표면의 z값이 0인경우 제거
                if (zz != 0) {
                    //표면과 보다 조인트의 거리가 카메라와 가까운 경우  제거
                    if (window3d.m_joint2[i].z > zz) {
                        //표면과 조인트의 거리가 15cm가 넘는 경우 제거
                        if (window3d.m_joint2[i].z - zz < 0.15) {
                            //뎁스 해상도를 넘어가는 조인트 제거
                            if (joint_y > 10) {
                                int joint_num = window3d.m_joint2[i].num;
                                // 손 발 제거 
                                if (joint_num != 8 && joint_num != 9 && joint_num != 10 && joint_num != 15 && joint_num != 16 && joint_num != 17 && joint_num != 28 && joint_num != 29 && joint_num != 30 && joint_num != 31 && joint_num != 32) {
                                    printf("add joint %d : x = %d, y = %d, z = %f, depthmap = %f\n", window3d.m_joint2[i].num, joint_x, joint_y, window3d.m_joint2[i].z, zz);

                                    if (window3d.m_joint2[i].num == 0) {
                                        joint_0_x = joint_x;
                                        joint_0_y = joint_y;
                                    }
                                    if (window3d.m_joint2[i].num == 18) {
                                        joint_18_x = joint_x;
                                        joint_18_y = joint_y;
                                    }
                                    if (window3d.m_joint2[i].num == 22) {
                                        joint_22_x = joint_x;
                                        joint_22_y = joint_y;
                                    }
                                    if (window3d.m_joint2[i].num == 12) {
                                        joint_12_x = joint_x;
                                        joint_12_y = joint_y;
                                    }
                                    if (window3d.m_joint2[i].num == 5) {
                                        joint_5_x = joint_x;
                                        joint_5_y = joint_y;
                                    }
                                    if (window3d.m_joint2[i].num == 6) {
                                        joint_6_x = joint_x;
                                        joint_6_y = joint_y;
                                    }
                                    if (window3d.m_joint2[i].num == 13) {
                                        joint_13_x = joint_x;
                                        joint_13_y = joint_y;
                                    }

                                    add_joint_cam2++;

                                    //cv::Point p;
                                    //p.x = joint_x + 5;
                                    //p.y = joint_y + 5;
                                    //
                                    //cv::circle(depth_map_clone2, cv::Point(joint_x, joint_y), 2, cv::Scalar(255, 255, 255), -1, 8, 0);
                                    //cv::putText(depth_map_clone2, to_string(window3d.m_joint2[i].num), p, 2, 0.4, cv::Scalar(255, 0, 0), 1, 1);
                            
                                    window3d.m_joint_result2.push_back({ window3d.m_joint2[i].num,window3d.m_joint2[i].x ,window3d.m_joint2[i].y ,window3d.m_joint2[i].z});

                                }
                                else {
                                    printf("remove joint %d : x = %d, y = %d, z = %f, depthmap = %f\n", window3d.m_joint2[i].num, joint_x, joint_y, window3d.m_joint2[i].z, zz);
                                }
                            }
                            else {
                                printf("remove joint %d : x = %d, y = %d, z = %f, depthmap = %f\n", window3d.m_joint2[i].num, joint_x, joint_y, window3d.m_joint2[i].z, zz);
                            }
                        }
                        else {
                            printf("remove joint %d : x = %d, y = %d, z = %f, depthmap = %f\n", window3d.m_joint2[i].num, joint_x, joint_y, window3d.m_joint2[i].z, zz);
                        }
                    }
                    else {
                        printf("remove joint %d : x = %d, y = %d, z = %f, depthmap = %f\n", window3d.m_joint2[i].num, joint_x, joint_y, window3d.m_joint2[i].z, zz);
                    }
                }
                else {
                    printf("remove joint %d : x = %d, y = %d, z = %f, depthmap = %f\n", window3d.m_joint2[i].num, joint_x, joint_y, window3d.m_joint2[i].z, zz);
                }
            }

            if (joint_18_x != 0 && joint_18_y != 0 && joint_22_x != 0 && joint_22_y != 0 && joint_12_x != 0 && joint_12_y != 0 && joint_5_x != 0 && joint_5_y != 0) {

                //line(depth_map_clone2, cv::Point(joint_18_x, joint_18_y), cv::Point(joint_22_x, joint_22_y), cv::Scalar::all(255), 1, 8, 0);
                //line(depth_map_clone2, cv::Point(joint_12_x, joint_12_y), cv::Point(joint_5_x, joint_5_y), cv::Scalar::all(255), 1, 8, 0);
              
                cv::Point p;
                cv::Point p2;

                p.x = joint_22_x + 15;
                p.y = joint_22_y - 15;
                double dy = (double)joint_22_y - (double)joint_18_y;
                double dx = (double)joint_22_x - (double)joint_18_x;

                p2.x = joint_5_x + 15;
                p2.y = joint_5_y - 15;
                double dy2 = (double)joint_5_y - (double)joint_12_y;
                double dx2 = (double)joint_5_x - (double)joint_12_x;

                double tehta = atan(dy / dx);
                double tehta2 = atan(dy2 / dx2);
              
                //cv::putText(depth_map_clone2, to_string(tehta), p, 2, 0.4, cv::Scalar(255, 255, 255), 1, 1);
                //cv::putText(depth_map_clone2, to_string(tehta2), p2, 2, 0.4, cv::Scalar(255, 255, 255), 1, 1);

                tetha_cam2 = std::fabs(tehta);
            }

            printf("------------------------- cam3 -----------------------------\n");

            joint_0_x = 0;
            joint_0_y = 0;
            joint_6_x = 0;
            joint_6_y = 0;
            joint_12_x = 0;
            joint_12_y = 0;
            joint_13_x = 0;
            joint_13_y = 0;
            joint_5_x = 0;
            joint_5_y = 0;
            joint_18_x = 0;
            joint_18_y = 0;
            joint_22_x = 0;
            joint_22_y = 0;

            for (int i = 0; i < joint_count3; i++) {
                int joint_x = int(floor((window3d.m_joint3[i].int_x) + 0.5));
                int joint_y = int(floor((window3d.m_joint3[i].int_y) + 0.5));

                if (joint_x < 0) {
                    continue;
                }

                if (joint_y < 0) {
                    continue;
                }

                float zz = (float)depth_map3.at<ushort>(joint_y, joint_x) / 1000;

                //표면의 z값이 0인경우 제거
                if (zz != 0) {
                    //표면과 보다 조인트의 거리가 카메라와 가까운 경우  제거
                    if (window3d.m_joint3[i].z > zz) {
                        //표면과 조인트의 거리가 15cm가 넘는 경우 제거
                        if (window3d.m_joint3[i].z - zz < 0.15) {
                            //뎁스 해상도를 넘어가는 조인트 제거
                            if (joint_y > 10) {
                                int joint_num = window3d.m_joint3[i].num;
                                // 손 발 제거 
                                if (joint_num != 8 && joint_num != 9 && joint_num != 10 && joint_num != 15 && joint_num != 16 && joint_num != 17 && joint_num != 28 && joint_num != 29 && joint_num != 30 && joint_num != 31 && joint_num != 32) {
                                    printf("add joint %d : x = %d, y = %d, z = %f, depthmap = %f\n", window3d.m_joint3[i].num, joint_x, joint_y, window3d.m_joint3[i].z, zz);

                                    if (window3d.m_joint3[i].num == 0) {
                                        joint_0_x = joint_x;
                                        joint_0_y = joint_y;
                                    }
                                    if (window3d.m_joint3[i].num == 18) {
                                        joint_18_x = joint_x;
                                        joint_18_y = joint_y;
                                    }
                                    if (window3d.m_joint3[i].num == 22) {
                                        joint_22_x = joint_x;
                                        joint_22_y = joint_y;
                                    }
                                    if (window3d.m_joint3[i].num == 12) {
                                        joint_12_x = joint_x;
                                        joint_12_y = joint_y;
                                    }
                                    if (window3d.m_joint3[i].num == 5) {
                                        joint_5_x = joint_x;
                                        joint_5_y = joint_y;
                                    }
                                    if (window3d.m_joint3[i].num == 6) {
                                        joint_6_x = joint_x;
                                        joint_6_y = joint_y;
                                    }
                                    if (window3d.m_joint3[i].num == 13) {
                                        joint_13_x = joint_x;
                                        joint_13_y = joint_y;
                                    }

                                    add_joint_cam3++;

                                    //cv::Point p;
                                    //p.x = joint_x + 5;
                                    //p.y = joint_y + 5;
                                    //
                                    //cv::circle(depth_map_clone3, cv::Point(joint_x, joint_y), 2, cv::Scalar(255, 255, 255), -1, 8, 0);
                                    //cv::putText(depth_map_clone3, to_string(window3d.m_joint3[i].num), p, 2, 0.4, cv::Scalar(255, 0, 0), 1, 1);
                                
                                    window3d.m_joint_result3.push_back({ window3d.m_joint3[i].num,window3d.m_joint3[i].x ,window3d.m_joint3[i].y ,window3d.m_joint3[i].z});

                                }
                                else {
                                    printf("remove joint %d : x = %d, y = %d, z = %f, depthmap = %f\n", window3d.m_joint3[i].num, joint_x, joint_y, window3d.m_joint3[i].z, zz);
                                }
                            }
                            else {
                                printf("remove joint %d : x = %d, y = %d, z = %f, depthmap = %f\n", window3d.m_joint3[i].num, joint_x, joint_y, window3d.m_joint3[i].z, zz);
                            }
                        }
                        else {
                            printf("remove joint %d : x = %d, y = %d, z = %f, depthmap = %f\n", window3d.m_joint3[i].num, joint_x, joint_y, window3d.m_joint3[i].z, zz);
                        }
                    }
                    else {
                        printf("remove joint %d : x = %d, y = %d, z = %f, depthmap = %f\n", window3d.m_joint3[i].num, joint_x, joint_y, window3d.m_joint3[i].z, zz);
                    }
                }
                else {
                    printf("remove joint %d : x = %d, y = %d, z = %f, depthmap = %f\n", window3d.m_joint3[i].num, joint_x, joint_y, window3d.m_joint3[i].z, zz);
                }
            }

            if (joint_18_x != 0 && joint_18_y != 0 && joint_22_x != 0 && joint_22_y != 0 && joint_12_x != 0 && joint_12_y != 0 && joint_5_x != 0 && joint_5_y != 0) {

                //line(depth_map_clone3, cv::Point(joint_18_x, joint_18_y), cv::Point(joint_22_x, joint_22_y), cv::Scalar::all(255), 1, 8, 0);
                //line(depth_map_clone3, cv::Point(joint_12_x, joint_12_y), cv::Point(joint_5_x, joint_5_y), cv::Scalar::all(255), 1, 8, 0);
           
                cv::Point p;
                cv::Point p2;
            
                p.x = joint_22_x + 15;
                p.y = joint_22_y - 15;
                double dy = (double)joint_22_y - (double)joint_18_y;
                double dx = (double)joint_22_x - (double)joint_18_x;

                p2.x = joint_5_x + 15;
                p2.y = joint_5_y - 15;
                double dy2 = (double)joint_5_y - (double)joint_12_y;
                double dx2 = (double)joint_5_x - (double)joint_12_x;


                double tehta = atan(dy / dx);
                double tehta2 = atan(dy2 / dx2);

                //cv::putText(depth_map_clone3, to_string(tehta), p, 2, 0.4, cv::Scalar(255, 255, 255), 1, 1);
                //cv::putText(depth_map_clone3, to_string(tehta2), p2, 2, 0.4, cv::Scalar(255, 255, 255), 1, 1);

                tetha_cam3 = std::fabs(tehta);
            }

            printf("------------------------- cam4 -----------------------------\n");

            joint_0_x = 0;
            joint_0_y = 0;
            joint_6_x = 0;
            joint_6_y = 0;
            joint_12_x = 0;
            joint_12_y = 0;
            joint_13_x = 0;
            joint_13_y = 0;
            joint_5_x = 0;
            joint_5_y = 0;
            joint_18_x = 0;
            joint_18_y = 0;
            joint_22_x = 0;
            joint_22_y = 0;

            for (int i = 0; i < joint_count4; i++) {
                int joint_x = int(floor((window3d.m_joint4[i].int_x) + 0.5));
                int joint_y = int(floor((window3d.m_joint4[i].int_y) + 0.5));

                if (joint_x < 0) {
                    continue;
                }

                if (joint_y < 0) {
                    continue;
                }

                float zz = (float)depth_map4.at<ushort>(joint_y, joint_x) / 1000;

                //표면의 z값이 0인경우 제거
                if (zz != 0) {
                    //표면과 보다 조인트의 거리가 카메라와 가까운 경우  제거
                    if (window3d.m_joint4[i].z > zz) {
                        //표면과 조인트의 거리가 15cm가 넘는 경우 제거
                        if (window3d.m_joint4[i].z - zz < 0.15) {
                            //뎁스 해상도를 넘어가는 조인트 제거
                            if (joint_y > 10) {
                                int joint_num = window3d.m_joint4[i].num;
                                // 손 발 제거 
                                if (joint_num != 8 && joint_num != 9 && joint_num != 10 && joint_num != 15 && joint_num != 16 && joint_num != 17 && joint_num != 28 && joint_num != 29 && joint_num != 30 && joint_num != 31 && joint_num != 32) {
                                    printf("add joint %d : x = %d, y = %d, z = %f, depthmap = %f\n", window3d.m_joint4[i].num, joint_x, joint_y, window3d.m_joint4[i].z, zz);

                                    if (window3d.m_joint4[i].num == 0) {
                                        joint_0_x = joint_x;
                                        joint_0_y = joint_y;
                                    }
                                    if (window3d.m_joint4[i].num == 18) {
                                        joint_18_x = joint_x;
                                        joint_18_y = joint_y;
                                    }
                                    if (window3d.m_joint4[i].num == 22) {
                                        joint_22_x = joint_x;
                                        joint_22_y = joint_y;
                                    }
                                    if (window3d.m_joint4[i].num == 12) {
                                        joint_12_x = joint_x;
                                        joint_12_y = joint_y;
                                    }
                                    if (window3d.m_joint4[i].num == 5) {
                                        joint_5_x = joint_x;
                                        joint_5_y = joint_y;
                                    }
                                    if (window3d.m_joint4[i].num == 6) {
                                        joint_6_x = joint_x;
                                        joint_6_y = joint_y;
                                    }
                                    if (window3d.m_joint4[i].num == 13) {
                                        joint_13_x = joint_x;
                                        joint_13_y = joint_y;
                                    }

                                    add_joint_cam4++;

                                    //cv::Point p;
                                    //p.x = joint_x + 5;
                                    //p.y = joint_y + 5;
                                    //
                                    //cv::circle(depth_map_clone4, cv::Point(joint_x, joint_y), 2, cv::Scalar(255, 255, 255), -1, 8, 0);
                                    //cv::putText(depth_map_clone4, to_string(window3d.m_joint4[i].num), p, 2, 0.4, cv::Scalar(255, 0, 0), 1, 1);
      
                                    window3d.m_joint_result4.push_back({ window3d.m_joint4[i].num,window3d.m_joint4[i].x ,window3d.m_joint4[i].y ,window3d.m_joint4[i].z});

                                }
                                else {
                                    printf("remove joint %d : x = %d, y = %d, z = %f, depthmap = %f\n", window3d.m_joint4[i].num, joint_x, joint_y, window3d.m_joint4[i].z, zz);
                                }
                            }
                            else {
                                printf("remove joint %d : x = %d, y = %d, z = %f, depthmap = %f\n", window3d.m_joint4[i].num, joint_x, joint_y, window3d.m_joint4[i].z, zz);
                            }
                        }
                        else {
                            printf("remove joint %d : x = %d, y = %d, z = %f, depthmap = %f\n", window3d.m_joint4[i].num, joint_x, joint_y, window3d.m_joint4[i].z, zz);
                        }
                    }
                    else {
                        printf("remove joint %d : x = %d, y = %d, z = %f, depthmap = %f\n", window3d.m_joint4[i].num, joint_x, joint_y, window3d.m_joint4[i].z, zz);
                    }
                }
                else {
                    printf("remove joint %d : x = %d, y = %d, z = %f, depthmap = %f\n", window3d.m_joint4[i].num, joint_x, joint_y, window3d.m_joint4[i].z, zz);
                }
            }

            if (joint_18_x != 0 && joint_18_y != 0 && joint_22_x != 0 && joint_22_y != 0 && joint_12_x != 0 && joint_12_y != 0 && joint_5_x != 0 && joint_5_y != 0) {

                //line(depth_map_clone4, cv::Point(joint_18_x, joint_18_y), cv::Point(joint_22_x, joint_22_y), cv::Scalar::all(255), 1, 8, 0);
                //line(depth_map_clone4, cv::Point(joint_12_x, joint_12_y), cv::Point(joint_5_x, joint_5_y), cv::Scalar::all(255), 1, 8, 0);
             
                cv::Point p;
                cv::Point p2;
            
                p.x = joint_22_x + 15;
                p.y = joint_22_y - 15;
                double dy = (double)joint_22_y - (double)joint_18_y;
                double dx = (double)joint_22_x - (double)joint_18_x;

                p2.x = joint_5_x + 15;
                p2.y = joint_5_y - 15;
                double dy2 = (double)joint_5_y - (double)joint_12_y;
                double dx2 = (double)joint_5_x - (double)joint_12_x;

                double tehta = atan(dy / dx);
                double tehta2 = atan(dy2 / dx2);

                //cv::putText(depth_map_clone4, to_string(tehta), p, 2, 0.4, cv::Scalar(255, 255, 255), 1, 1);
                //cv::putText(depth_map_clone4, to_string(tehta2), p2, 2, 0.4, cv::Scalar(255, 255, 255), 1, 1);

                tetha_cam4 = std::fabs(tehta);
            }

            printf("------------------------- cam5 -----------------------------\n");

            joint_0_x = 0;
            joint_0_y = 0;
            joint_6_x = 0;
            joint_6_y = 0;
            joint_12_x = 0;
            joint_12_y = 0;
            joint_13_x = 0;
            joint_13_y = 0;
            joint_5_x = 0;
            joint_5_y = 0;
            joint_18_x = 0;
            joint_18_y = 0;
            joint_22_x = 0;
            joint_22_y = 0;
          
            for (int i = 0; i < joint_count5; i++) {
                int joint_x = int(floor((window3d.m_joint5[i].int_x) + 0.5));
                int joint_y = int(floor((window3d.m_joint5[i].int_y) + 0.5));

                if (joint_x < 0) {
                    continue;
                }

                if (joint_y < 0) {
                    continue;
                }

                float zz = (float)depth_map5.at<ushort>(joint_y, joint_x) / 1000;

                //표면의 z값이 0인경우 제거
                if (zz != 0) {
                    //표면과 보다 조인트의 거리가 카메라와 가까운 경우  제거
                    if (window3d.m_joint5[i].z > zz) {
                        //표면과 조인트의 거리가 15cm가 넘는 경우 제거
                        if (window3d.m_joint5[i].z - zz < 0.15) {
                            //뎁스 해상도를 넘어가는 조인트 제거
                            if (joint_y > 10) {
                                int joint_num = window3d.m_joint5[i].num;
                                // 손 발 제거 
                                if (joint_num != 8 && joint_num != 9 && joint_num != 10 && joint_num != 15 && joint_num != 16 && joint_num != 17 && joint_num != 28 && joint_num != 29 && joint_num != 30 && joint_num != 31 && joint_num != 32) {
                                    printf("add joint %d : x = %d, y = %d, z = %f, depthmap = %f\n", window3d.m_joint5[i].num, joint_x, joint_y, window3d.m_joint5[i].z, zz);

                                    if (window3d.m_joint5[i].num == 0) {
                                        joint_0_x = joint_x;
                                        joint_0_y = joint_y;
                                    }
                                    if (window3d.m_joint5[i].num == 18) {
                                        joint_18_x = joint_x;
                                        joint_18_y = joint_y;
                                    }
                                    if (window3d.m_joint5[i].num == 22) {
                                        joint_22_x = joint_x;
                                        joint_22_y = joint_y;
                                    }
                                    if (window3d.m_joint5[i].num == 12) {
                                        joint_12_x = joint_x;
                                        joint_12_y = joint_y;
                                    }
                                    if (window3d.m_joint5[i].num == 5) {
                                        joint_5_x = joint_x;
                                        joint_5_y = joint_y;
                                    }
                                    if (window3d.m_joint5[i].num == 6) {
                                        joint_6_x = joint_x;
                                        joint_6_y = joint_y;
                                    }
                                    if (window3d.m_joint5[i].num == 13) {
                                        joint_13_x = joint_x;
                                        joint_13_y = joint_y;
                                    }

                                    add_joint_cam5++;

                                    //cv::Point p;
                                    //p.x = joint_x + 5;
                                    //p.y = joint_y + 5;
                                    //
                                    //cv::circle(depth_map_clone5, cv::Point(joint_x, joint_y), 2, cv::Scalar(255, 255, 255), -1, 8, 0);
                                    //cv::putText(depth_map_clone5, to_string(window3d.m_joint5[i].num), p, 2, 0.4, cv::Scalar(255, 0, 0), 1, 1);

                                    window3d.m_joint_result5.push_back({ window3d.m_joint5[i].num,window3d.m_joint5[i].x ,window3d.m_joint5[i].y ,window3d.m_joint5[i].z});

                                }
                                else {
                                    printf("remove joint %d : x = %d, y = %d, z = %f, depthmap = %f\n", window3d.m_joint5[i].num, joint_x, joint_y, window3d.m_joint5[i].z, zz);
                                }
                            }
                            else {
                                printf("remove joint %d : x = %d, y = %d, z = %f, depthmap = %f\n", window3d.m_joint5[i].num, joint_x, joint_y, window3d.m_joint5[i].z, zz);
                            }
                        }
                        else {
                            printf("remove joint %d : x = %d, y = %d, z = %f, depthmap = %f\n", window3d.m_joint5[i].num, joint_x, joint_y, window3d.m_joint5[i].z, zz);
                        }
                    }
                    else {
                        printf("remove joint %d : x = %d, y = %d, z = %f, depthmap = %f\n", window3d.m_joint5[i].num, joint_x, joint_y, window3d.m_joint5[i].z, zz);
                    }
                }
                else {
                    printf("remove joint %d : x = %d, y = %d, z = %f, depthmap = %f\n", window3d.m_joint5[i].num, joint_x, joint_y, window3d.m_joint5[i].z, zz);
                }
            }

            if (joint_18_x != 0 && joint_18_y != 0 && joint_22_x != 0 && joint_22_y != 0 && joint_12_x != 0 && joint_12_y != 0 && joint_5_x != 0 && joint_5_y != 0) {

                //line(depth_map_clone5, cv::Point(joint_18_x, joint_18_y), cv::Point(joint_22_x, joint_22_y), cv::Scalar::all(255), 1, 8, 0);
                //line(depth_map_clone5, cv::Point(joint_12_x, joint_12_y), cv::Point(joint_5_x, joint_5_y), cv::Scalar::all(255), 1, 8, 0);
           
                cv::Point p;
                cv::Point p2;
               
                p.x = joint_22_x + 15;
                p.y = joint_22_y - 15;
                double dy = (double)joint_22_y - (double)joint_18_y;
                double dx = (double)joint_22_x - (double)joint_18_x;

                p2.x = joint_5_x + 15;
                p2.y = joint_5_y - 15;
                double dy2 = (double)joint_5_y - (double)joint_12_y;
                double dx2 = (double)joint_5_x - (double)joint_12_x;
                
                double tehta = atan(dy / dx);
                double tehta2 = atan(dy2 / dx2);
               
                //cv::putText(depth_map_clone5, to_string(tehta), p, 2, 0.4, cv::Scalar(255, 255, 255), 1, 1);
                //cv::putText(depth_map_clone5, to_string(tehta2), p2, 2, 0.4, cv::Scalar(255, 255, 255), 1, 1);
                
                tetha_cam5 = std::fabs(tehta);
            }

            printf("------------------------- cam6 -----------------------------\n");

            joint_0_x = 0;
            joint_0_y = 0;
            joint_6_x = 0;
            joint_6_y = 0;
            joint_12_x = 0;
            joint_12_y = 0;
            joint_13_x = 0;
            joint_13_y = 0;
            joint_5_x = 0;
            joint_5_y = 0;
            joint_18_x = 0;
            joint_18_y = 0;
            joint_22_x = 0;
            joint_22_y = 0;

            for (int i = 0; i < joint_count6; i++) {
                int joint_x = int(floor((window3d.m_joint6[i].int_x) + 0.5));
                int joint_y = int(floor((window3d.m_joint6[i].int_y) + 0.5));

                if (joint_x < 0) {
                    continue;
                }

                if (joint_y < 0) {
                    continue;
                }

                float zz = (float)depth_map6.at<ushort>(joint_y, joint_x) / 1000;

                //표면의 z값이 0인경우 제거
                if (zz != 0) {
                    //표면과 보다 조인트의 거리가 카메라와 가까운 경우  제거
                    if (window3d.m_joint6[i].z > zz) {
                        //표면과 조인트의 거리가 15cm가 넘는 경우 제거
                        if (window3d.m_joint6[i].z - zz < 0.15) {
                            //뎁스 해상도를 넘어가는 조인트 제거
                            if (joint_y > 10) {
                                int joint_num = window3d.m_joint6[i].num;
                                // 손 발 제거 
                                if (joint_num != 8 && joint_num != 9 && joint_num != 10 && joint_num != 15 && joint_num != 16 && joint_num != 17 && joint_num != 28 && joint_num != 29 && joint_num != 30 && joint_num != 31 && joint_num != 32) {
                                    printf("add joint %d : x = %d, y = %d, z = %f, depthmap = %f\n", window3d.m_joint6[i].num, joint_x, joint_y, window3d.m_joint6[i].z, zz);

                                    if (window3d.m_joint6[i].num == 0) {
                                        joint_0_x = joint_x;
                                        joint_0_y = joint_y;
                                    }
                                    if (window3d.m_joint6[i].num == 18) {
                                        joint_18_x = joint_x;
                                        joint_18_y = joint_y;
                                    }
                                    if (window3d.m_joint6[i].num == 22) {
                                        joint_22_x = joint_x;
                                        joint_22_y = joint_y;
                                    }
                                    if (window3d.m_joint6[i].num == 12) {
                                        joint_12_x = joint_x;
                                        joint_12_y = joint_y;
                                    }
                                    if (window3d.m_joint6[i].num == 5) {
                                        joint_5_x = joint_x;
                                        joint_5_y = joint_y;
                                    }
                                    if (window3d.m_joint6[i].num == 6) {
                                        joint_6_x = joint_x;
                                        joint_6_y = joint_y;
                                    }
                                    if (window3d.m_joint6[i].num == 13) {
                                        joint_13_x = joint_x;
                                        joint_13_y = joint_y;
                                    }

                                    add_joint_cam6++;

                                    //cv::Point p;
                                    //p.x = joint_x + 5;
                                    //p.y = joint_y + 5;
                                    //
                                    //cv::circle(depth_map_clone6, cv::Point(joint_x, joint_y), 2, cv::Scalar(255, 255, 255), -1, 8, 0);
                                    //cv::putText(depth_map_clone6, to_string(window3d.m_joint6[i].num), p, 2, 0.4, cv::Scalar(255, 0, 0), 1, 1);
                                
                                    window3d.m_joint_result6.push_back({ window3d.m_joint6[i].num,window3d.m_joint6[i].x ,window3d.m_joint6[i].y ,window3d.m_joint6[i].z});

                                }
                                else {
                                    printf("remove joint %d : x = %d, y = %d, z = %f, depthmap = %f\n", window3d.m_joint6[i].num, joint_x, joint_y, window3d.m_joint6[i].z, zz);
                                }
                            }
                            else {
                                printf("remove joint %d : x = %d, y = %d, z = %f, depthmap = %f\n", window3d.m_joint6[i].num, joint_x, joint_y, window3d.m_joint6[i].z, zz);
                            }
                        }
                        else {
                            printf("remove joint %d : x = %d, y = %d, z = %f, depthmap = %f\n", window3d.m_joint6[i].num, joint_x, joint_y, window3d.m_joint6[i].z, zz);
                        }
                    }
                    else {
                        printf("remove joint %d : x = %d, y = %d, z = %f, depthmap = %f\n", window3d.m_joint6[i].num, joint_x, joint_y, window3d.m_joint6[i].z, zz);
                    }
                }
                else {
                    printf("remove joint %d : x = %d, y = %d, z = %f, depthmap = %f\n", window3d.m_joint6[i].num, joint_x, joint_y, window3d.m_joint6[i].z, zz);
                }
            }

            if (joint_18_x != 0 && joint_18_y != 0 && joint_22_x != 0 && joint_22_y != 0 && joint_12_x != 0 && joint_12_y != 0 && joint_5_x != 0 && joint_5_y != 0) {

                //line(depth_map_clone6, cv::Point(joint_18_x, joint_18_y), cv::Point(joint_22_x, joint_22_y), cv::Scalar::all(255), 1, 8, 0);
                //line(depth_map_clone6, cv::Point(joint_12_x, joint_12_y), cv::Point(joint_5_x, joint_5_y), cv::Scalar::all(255), 1, 8, 0);
          
                cv::Point p;
                cv::Point p2;
               
                p.x = joint_22_x + 15;
                p.y = joint_22_y - 15;
                double dy = (double)joint_22_y - (double)joint_18_y;
                double dx = (double)joint_22_x - (double)joint_18_x;

                p2.x = joint_5_x + 15;
                p2.y = joint_5_y - 15;
                double dy2 = (double)joint_5_y - (double)joint_12_y;
                double dx2 = (double)joint_5_x - (double)joint_12_x;

                double tehta = atan(dy / dx);
                double tehta2 = atan(dy2 / dx2);

                //cv::putText(depth_map_clone6, to_string(tehta), p, 2, 0.4, cv::Scalar(255, 255, 255), 1, 1);
                //cv::putText(depth_map_clone6, to_string(tehta2), p2, 2, 0.4, cv::Scalar(255, 255, 255), 1, 1);

                tetha_cam6 = std::fabs(tehta);
            }

            printf("------------------------- cam7 -----------------------------\n");

            joint_0_x = 0;
            joint_0_y = 0;
            joint_6_x = 0;
            joint_6_y = 0;
            joint_12_x = 0;
            joint_12_y = 0;
            joint_13_x = 0;
            joint_13_y = 0;
            joint_5_x = 0;
            joint_5_y = 0;
            joint_18_x = 0;
            joint_18_y = 0;
            joint_22_x = 0;
            joint_22_y = 0;

            for (int i = 0; i < joint_count7; i++) {
                int joint_x = int(floor((window3d.m_joint7[i].int_x) + 0.5));
                int joint_y = int(floor((window3d.m_joint7[i].int_y) + 0.5));

                if (joint_x < 0) {
                    continue;
                }

                if (joint_y < 0) {
                    continue;
                }

                float zz = (float)depth_map7.at<ushort>(joint_y, joint_x) / 1000;

                //표면의 z값이 0인경우 제거
                if (zz != 0) {
                    //표면과 보다 조인트의 거리가 카메라와 가까운 경우  제거
                    if (window3d.m_joint7[i].z > zz) {
                        //표면과 조인트의 거리가 15cm가 넘는 경우 제거
                        if (window3d.m_joint7[i].z - zz < 0.15) {
                            //뎁스 해상도를 넘어가는 조인트 제거
                            if (joint_y > 10) {
                                int joint_num = window3d.m_joint7[i].num;
                                // 손 발 제거 
                                if (joint_num != 8 && joint_num != 9 && joint_num != 10 && joint_num != 15 && joint_num != 16 && joint_num != 17 && joint_num != 28 && joint_num != 29 && joint_num != 30 && joint_num != 31 && joint_num != 32) {
                                    printf("add joint %d : x = %d, y = %d, z = %f, depthmap = %f\n", window3d.m_joint7[i].num, joint_x, joint_y, window3d.m_joint7[i].z, zz);

                                    if (window3d.m_joint7[i].num == 0) {
                                        joint_0_x = joint_x;
                                        joint_0_y = joint_y;
                                    }
                                    if (window3d.m_joint7[i].num == 18) {
                                        joint_18_x = joint_x;
                                        joint_18_y = joint_y;
                                    }
                                    if (window3d.m_joint7[i].num == 22) {
                                        joint_22_x = joint_x;
                                        joint_22_y = joint_y;
                                    }
                                    if (window3d.m_joint7[i].num == 12) {
                                        joint_12_x = joint_x;
                                        joint_12_y = joint_y;
                                    }
                                    if (window3d.m_joint7[i].num == 5) {
                                        joint_5_x = joint_x;
                                        joint_5_y = joint_y;
                                    }
                                    if (window3d.m_joint7[i].num == 6) {
                                        joint_6_x = joint_x;
                                        joint_6_y = joint_y;
                                    }
                                    if (window3d.m_joint7[i].num == 13) {
                                        joint_13_x = joint_x;
                                        joint_13_y = joint_y;
                                    }

                                    add_joint_cam7++;

                                    //cv::Point p;
                                    //p.x = joint_x + 5;
                                    //p.y = joint_y + 5;
                                    //
                                    //cv::circle(depth_map_clone7, cv::Point(joint_x, joint_y), 2, cv::Scalar(255, 255, 255), -1, 8, 0);
                                    //cv::putText(depth_map_clone7, to_string(window3d.m_joint7[i].num), p, 2, 0.4, cv::Scalar(255, 0, 0), 1, 1);

                                    window3d.m_joint_result7.push_back({ window3d.m_joint7[i].num,window3d.m_joint7[i].x ,window3d.m_joint7[i].y ,window3d.m_joint7[i].z});

                                }
                                else {
                                    printf("remove joint %d : x = %d, y = %d, z = %f, depthmap = %f\n", window3d.m_joint7[i].num, joint_x, joint_y, window3d.m_joint7[i].z, zz);
                                }
                            }
                            else {
                                printf("remove joint %d : x = %d, y = %d, z = %f, depthmap = %f\n", window3d.m_joint7[i].num, joint_x, joint_y, window3d.m_joint7[i].z, zz);
                            }
                        }
                        else {
                            printf("remove joint %d : x = %d, y = %d, z = %f, depthmap = %f\n", window3d.m_joint7[i].num, joint_x, joint_y, window3d.m_joint7[i].z, zz);
                        }
                    }
                    else {
                        printf("remove joint %d : x = %d, y = %d, z = %f, depthmap = %f\n", window3d.m_joint7[i].num, joint_x, joint_y, window3d.m_joint7[i].z, zz);
                    }
                }
                else {
                    printf("remove joint %d : x = %d, y = %d, z = %f, depthmap = %f\n", window3d.m_joint7[i].num, joint_x, joint_y, window3d.m_joint7[i].z, zz);
                }
            }

            if (joint_18_x != 0 && joint_18_y != 0 && joint_22_x != 0 && joint_22_y != 0 && joint_12_x != 0 && joint_12_y != 0 && joint_5_x != 0 && joint_5_y != 0) {

                //line(depth_map_clone7, cv::Point(joint_18_x, joint_18_y), cv::Point(joint_22_x, joint_22_y), cv::Scalar::all(255), 1, 8, 0);
                //line(depth_map_clone7, cv::Point(joint_12_x, joint_12_y), cv::Point(joint_5_x, joint_5_y), cv::Scalar::all(255), 1, 8, 0);
            
                cv::Point p;
                cv::Point p2;
               
                p.x = joint_22_x + 15;
                p.y = joint_22_y - 15;
                double dy = (double)joint_22_y - (double)joint_18_y;
                double dx = (double)joint_22_x - (double)joint_18_x;

                p2.x = joint_5_x + 15;
                p2.y = joint_5_y - 15;
                double dy2 = (double)joint_5_y - (double)joint_12_y;
                double dx2 = (double)joint_5_x - (double)joint_12_x;


                double tehta = atan(dy / dx);
                double tehta2 = atan(dy2 / dx2);
                
                //cv::putText(depth_map_clone7, to_string(tehta), p, 2, 0.4, cv::Scalar(255, 255, 255), 1, 1);
                //cv::putText(depth_map_clone7, to_string(tehta2), p2, 2, 0.4, cv::Scalar(255, 255, 255), 1, 1);
                
                tetha_cam7 = std::fabs(tehta);
            }

            printf("------------------------- cam8 -----------------------------\n");

            joint_0_x = 0;
            joint_0_y = 0;
            joint_6_x = 0;
            joint_6_y = 0;
            joint_12_x = 0;
            joint_12_y = 0;
            joint_13_x = 0;
            joint_13_y = 0;
            joint_5_x = 0;
            joint_5_y = 0;
            joint_18_x = 0;
            joint_18_y = 0;
            joint_22_x = 0;
            joint_22_y = 0;
            //window3d.m_joint_result8.push_back({ iteration, });

            for (int i = 0; i < joint_count8; i++) {
                int joint_x = int(floor((window3d.m_joint8[i].int_x) + 0.5));
                int joint_y = int(floor((window3d.m_joint8[i].int_y) + 0.5));

                if (joint_x < 0) {
                    continue;
                }

                if (joint_y < 0) {
                    continue;
                }

                float zz = (float)depth_map8.at<ushort>(joint_y, joint_x) / 1000;

                //표면의 z값이 0인경우 제거
                if (zz != 0) {
                    //표면과 보다 조인트의 거리가 카메라와 가까운 경우  제거
                    if (window3d.m_joint8[i].z > zz) {
                        //표면과 조인트의 거리가 15cm가 넘는 경우 제거
                        if (window3d.m_joint8[i].z - zz < 0.15) {
                            //뎁스 해상도를 넘어가는 조인트 제거
                            if (joint_y > 10) {
                                int joint_num = window3d.m_joint8[i].num;
                                // 손 발 제거 
                                if (joint_num != 8 && joint_num != 9 && joint_num != 10 && joint_num != 15 && joint_num != 16 && joint_num != 17 && joint_num != 28 && joint_num != 29 && joint_num != 30 && joint_num != 31 && joint_num != 32) {
                                    printf("add joint %d : x = %d, y = %d, z = %f, depthmap = %f\n", window3d.m_joint8[i].num, joint_x, joint_y, window3d.m_joint8[i].z, zz);

                                    if (window3d.m_joint8[i].num == 0) {
                                        joint_0_x = joint_x;
                                        joint_0_y = joint_y;
                                    }
                                    if (window3d.m_joint8[i].num == 18) {
                                        joint_18_x = joint_x;
                                        joint_18_y = joint_y;
                                    }
                                    if (window3d.m_joint8[i].num == 22) {
                                        joint_22_x = joint_x;
                                        joint_22_y = joint_y;
                                    }
                                    if (window3d.m_joint8[i].num == 12) {
                                        joint_12_x = joint_x;
                                        joint_12_y = joint_y;
                                    }
                                    if (window3d.m_joint8[i].num == 5) {
                                        joint_5_x = joint_x;
                                        joint_5_y = joint_y;
                                    }
                                    if (window3d.m_joint8[i].num == 6) {
                                        joint_6_x = joint_x;
                                        joint_6_y = joint_y;
                                    }
                                    if (window3d.m_joint8[i].num == 13) {
                                        joint_13_x = joint_x;
                                        joint_13_y = joint_y;
                                    }

                                    add_joint_cam8++;

                                    //cv::Point p;
                                    //p.x = joint_x + 5;
                                    //p.y = joint_y + 5;
                                    //
                                    //cv::circle(depth_map_clone8, cv::Point(joint_x, joint_y), 2, cv::Scalar(255, 255, 255), -1, 8, 0);
                                    //cv::putText(depth_map_clone8, to_string(window3d.m_joint8[i].num), p, 2, 0.4, cv::Scalar(255, 0, 0), 1, 1);

                                    window3d.m_joint_result8.push_back({ window3d.m_joint8[i].num,window3d.m_joint8[i].x ,window3d.m_joint8[i].y ,window3d.m_joint8[i].z});

                                }
                                else {
                                    printf("remove joint %d : x = %d, y = %d, z = %f, depthmap = %f\n", window3d.m_joint8[i].num, joint_x, joint_y, window3d.m_joint8[i].z, zz);
                                }
                            }
                            else {
                                printf("remove joint %d : x = %d, y = %d, z = %f, depthmap = %f\n", window3d.m_joint8[i].num, joint_x, joint_y, window3d.m_joint8[i].z, zz);
                            }
                        }
                        else {
                            printf("remove joint %d : x = %d, y = %d, z = %f, depthmap = %f\n", window3d.m_joint8[i].num, joint_x, joint_y, window3d.m_joint8[i].z, zz);
                        }
                    }
                    else {
                        printf("remove joint %d : x = %d, y = %d, z = %f, depthmap = %f\n", window3d.m_joint8[i].num, joint_x, joint_y, window3d.m_joint8[i].z, zz);
                    }
                }
                else {
                    printf("remove joint %d : x = %d, y = %d, z = %f, depthmap = %f\n", window3d.m_joint8[i].num, joint_x, joint_y, window3d.m_joint8[i].z, zz);
                }
            }

            if (joint_18_x != 0 && joint_18_y != 0 && joint_22_x != 0 && joint_22_y != 0 && joint_12_x != 0 && joint_12_y != 0 && joint_5_x != 0 && joint_5_y != 0) {

                //line(depth_map_clone8, cv::Point(joint_18_x, joint_18_y), cv::Point(joint_22_x, joint_22_y), cv::Scalar::all(255), 1, 8, 0);
                //line(depth_map_clone8, cv::Point(joint_12_x, joint_12_y), cv::Point(joint_5_x, joint_5_y), cv::Scalar::all(255), 1, 8, 0);
    
                cv::Point p;
                cv::Point p2;
                
                p.x = joint_22_x + 15;
                p.y = joint_22_y - 15;
                double dy = (double)joint_22_y - (double)joint_18_y;
                double dx = (double)joint_22_x - (double)joint_18_x;

                p2.x = joint_5_x + 15;
                p2.y = joint_5_y - 15;
                double dy2 = (double)joint_5_y - (double)joint_12_y;
                double dx2 = (double)joint_5_x - (double)joint_12_x;

                double tehta = atan(dy / dx);
                double tehta2 = atan(dy2 / dx2);

                //cv::putText(depth_map_clone8, to_string(tehta), p, 2, 0.4, cv::Scalar(255, 255, 255), 1, 1);
                //cv::putText(depth_map_clone8, to_string(tehta2), p2, 2, 0.4, cv::Scalar(255, 255, 255), 1, 1);
             
                tetha_cam8 = std::fabs(tehta);
            }

            int count_theta = 0;
            int count_joint = 0;
            
            
            //if (tetha_cam1 != 100) {
            //    count_theta++;
            //}
            //if (tetha_cam2 != 100) {
            //    count_theta++;
            //}
            //if (tetha_cam3 != 100) {
            //    count_theta++;
            //}
            //if (tetha_cam4 != 100) {
            //    count_theta++;
            //}
            //if (tetha_cam5 != 100) {
            //    count_theta++;
            //}
            //if (tetha_cam6 != 100) {
            //    count_theta++;
            //}
            //if (tetha_cam7 != 100) {
            //    count_theta++;
            //}
            //if (tetha_cam8 != 100) {
            //    count_theta++;
            //}
           
            int loss_sum = 0;
            for (int loss_size = 0; loss_size < loss_value.size(); loss_size++) {
                printf("cam %d - %f\n", loss_size, loss_value[loss_size]);
                if (loss_value[loss_size] < 0.05) {
                    loss_sum++;
                }
            }

           
            if (add_joint_cam1 > 5) {
                count_joint++;
            }
            if (add_joint_cam2 > 5) {
                count_joint++;
            }
            if (add_joint_cam3 > 5) {
                count_joint++;
            }
            if (add_joint_cam4 > 5) {
                count_joint++;
            }
            if (add_joint_cam5 > 5) {
                count_joint++;
            }
            if (add_joint_cam6 > 5) {
                count_joint++;
            }
            if (add_joint_cam7 > 5) {
                count_joint++;
            }
            if (add_joint_cam8 > 5) {
                count_joint++;
            }
           

            //printf("%d\n",count_theta);


            //if(count_joint > 3){
            if (count_joint > 5) {

                //if (loss_sum == 8) {
                //    continue;
                //}

                cv::Point p;
                p.x = 20 + 5;
                p.y = 20 + 5;

                std::vector<int> values;

                values.push_back(add_joint_cam1);
                values.push_back(add_joint_cam2);
                values.push_back(add_joint_cam3);
                values.push_back(add_joint_cam4);
                values.push_back(add_joint_cam5);
                values.push_back(add_joint_cam6);
                values.push_back(add_joint_cam7);
                values.push_back(add_joint_cam8);

                int max = 0;
                int temp_index = 0;
                int temp_index2 = 0;



                printf("values size : %d\n", values.size());

                for (int i = 0; i < values.size(); i++)
                {
                    if (max < values[i]){
                        max = values[i];
                        temp_index = i;
                        printf("max : %d , %d\n", max, i);
                    }
                }

                //temp_index = 1;

                /*
                if (temp_index == 0) {
                    cv::Point p;
                    p.x = 20 + 5;
                    p.y = 40 + 5;
                    cv::putText(depth_map_clone, "ref", p, 2, 0.4, cv::Scalar(255, 255, 255), 1, 1);                   
                }
                if (temp_index == 1) {
                    cv::Point p;
                    p.x = 20 + 5;
                    p.y = 40 + 5;
                    cv::putText(depth_map_clone2, "ref", p, 2, 0.4, cv::Scalar(255, 255, 255), 1, 1);
                }
                if (temp_index == 2) {
                    cv::Point p;
                    p.x = 20 + 5;
                    p.y = 40 + 5;
                    cv::putText(depth_map_clone3, "ref", p, 2, 0.4, cv::Scalar(255, 255, 255), 1, 1);
                }
                if (temp_index == 3) {
                    cv::Point p;
                    p.x = 20 + 5;
                    p.y = 40 + 5;
                    cv::putText(depth_map_clone4, "ref", p, 2, 0.4, cv::Scalar(255, 255, 255), 1, 1);
                }
                if (temp_index == 4) {
                    cv::Point p;
                    p.x = 20 + 5;
                    p.y = 40 + 5;
                    cv::putText(depth_map_clone5, "ref", p, 2, 0.4, cv::Scalar(255, 255, 255), 1, 1);
                }
                if (temp_index == 5) {
                    cv::Point p;
                    p.x = 20 + 5;
                    p.y = 40 + 5;
                    cv::putText(depth_map_clone6, "ref", p, 2, 0.4, cv::Scalar(255, 255, 255), 1, 1);
                }
                if (temp_index == 6) {
                    cv::Point p;
                    p.x = 20 + 5;
                    p.y = 40 + 5;
                    cv::putText(depth_map_clone7, "ref", p, 2, 0.4, cv::Scalar(255, 255, 255), 1, 1);

                }
                if (temp_index == 7) {
                    cv::Point p;
                    p.x = 20 + 5;
                    p.y = 40 + 5;
                    cv::putText(depth_map_clone8, "ref", p, 2, 0.4, cv::Scalar(255, 255, 255), 1, 1);
                }
                */

                int src_count = 0;


                std::vector<calib_idx> calib_order;

                if (tetha_cam1 < 0.1) {
                //cv::putText(depth_map_clone, to_string(tetha_cam1), p, 2, 0.4, cv::Scalar(255, 255, 255), 1, 1);
                //cv::imwrite("cam1_"+ to_string(iteration) +".bmp", depth_map_clone);
                temp_index2 = 1;
                calib_order.push_back({ iteration,temp_index + 1,temp_index2, (float)tetha_cam1 });
                //src_count++;
                }
                if (tetha_cam2 < 0.1) {
                //cv::putText(depth_map_clone2, to_string(tetha_cam2), p, 2, 0.4, cv::Scalar(255, 255, 255), 1, 1);
                //cv::imwrite("cam2_"+to_string(iteration)+".bmp", depth_map_clone2);
                temp_index2 = 2;
                calib_order.push_back({ iteration,temp_index + 1,temp_index2, (float)tetha_cam2 });
                //src_count++;
                }
                if (tetha_cam3 < 0.1) {
                //cv::putText(depth_map_clone3, to_string(tetha_cam3), p, 2, 0.4, cv::Scalar(255, 255, 255), 1, 1);
                //cv::imwrite("cam3_"+ to_string(iteration) +".bmp", depth_map_clone3);
                temp_index2 = 3;
                calib_order.push_back({ iteration,temp_index + 1,temp_index2,(float)tetha_cam3 });
                // src_count++;
                }
                if (tetha_cam4 < 0.1) {
                //cv::putText(depth_map_clone4, to_string(tetha_cam4), p, 2, 0.4, cv::Scalar(255, 255, 255), 1, 1);
                //cv::imwrite("cam4_"+ to_string(iteration) +".bmp", depth_map_clone4);
                temp_index2 = 4;
                calib_order.push_back({ iteration,temp_index + 1,temp_index2, (float)tetha_cam4 });
                // src_count++;
                }
                if (tetha_cam5 < 0.1) {
                //cv::putText(depth_map_clone5, to_string(tetha_cam5), p, 2, 0.4, cv::Scalar(255, 255, 255), 1, 1);
                //cv::imwrite("cam5_"+ to_string(iteration) +".bmp", depth_map_clone5);
                temp_index2 = 5;
                calib_order.push_back({ iteration,temp_index + 1,temp_index2, (float)tetha_cam5 });
                //src_count++;
                }
                if (tetha_cam6 < 0.1) {
                //cv::putText(depth_map_clone6, to_string(tetha_cam6), p, 2, 0.4, cv::Scalar(255, 255, 255), 1, 1);
                //cv::imwrite("cam6_"+ to_string(iteration) +".bmp", depth_map_clone6);
                temp_index2 = 6;
                calib_order.push_back({ iteration,temp_index + 1,temp_index2, (float)tetha_cam6 });
                //src_count++;
                }
                if (tetha_cam7 < 0.1) {
                //cv::putText(depth_map_clone7, to_string(tetha_cam7), p, 2, 0.4, cv::Scalar(255, 255, 255), 1, 1);
                //cv::imwrite("cam7_"+ to_string(iteration) +".bmp", depth_map_clone7);
                temp_index2 = 7;
                calib_order.push_back({ iteration,temp_index + 1,temp_index2, (float)tetha_cam7 });
                //src_count++;
                }
                if (tetha_cam8 < 0.1) {
                //cv::putText(depth_map_clone8, to_string(tetha_cam8), p, 2, 0.4, cv::Scalar(255, 255, 255), 1, 1);
                //cv::imwrite("cam8_"+ to_string(iteration) +".bmp", depth_map_clone8);
                temp_index2 = 8;
                calib_order.push_back({ iteration,temp_index + 1,temp_index2, (float)tetha_cam8 });
                //src_count++;
                }

                if (calib_order.size() > 3) {

                    for (int ii = 0; ii < calib_order.size(); ii++) {
                        if(calib_order[ii].ref_num != calib_order[ii].cam_num){
                            printf("frame num : %d, ref num : %d, src num :%d, theta : %f\n", calib_order[ii].frame_num, calib_order[ii].ref_num, calib_order[ii].cam_num, calib_order[ii].theta);

                            int opti_count = 0;

                            int ref_idx = calib_order[ii].ref_num;
                            int src_idx = calib_order[ii].cam_num;


                            if (ref_idx == 1) {
                                window3d.m_joint_result_ref = window3d.m_joint;
                            }
                            if (ref_idx == 2) {
                                window3d.m_joint_result_ref = window3d.m_joint2;
                            }
                            if (ref_idx == 3) {
                                window3d.m_joint_result_ref = window3d.m_joint3;
                            }
                            if (ref_idx == 4) {
                                window3d.m_joint_result_ref = window3d.m_joint4;
                            }
                            if (ref_idx == 5) {
                                window3d.m_joint_result_ref = window3d.m_joint5;
                            }
                            if (ref_idx == 6) {
                                window3d.m_joint_result_ref = window3d.m_joint6;
                            }
                            if (ref_idx == 7) {
                                window3d.m_joint_result_ref = window3d.m_joint7;
                            }
                            if (ref_idx == 8) {
                                window3d.m_joint_result_ref = window3d.m_joint8;
                            }

                            if (src_idx == 1) {
                                window3d.m_joint_result_src = window3d.m_joint;
                            }
                            if (src_idx == 2) {
                                window3d.m_joint_result_src = window3d.m_joint2;
                            }
                            if (src_idx == 3) {
                                window3d.m_joint_result_src = window3d.m_joint3;
                            }
                            if (src_idx == 4) {
                                window3d.m_joint_result_src = window3d.m_joint4;
                            }
                            if (src_idx == 5) {
                                window3d.m_joint_result_src = window3d.m_joint5;
                            }
                            if (src_idx == 6) {
                                window3d.m_joint_result_src = window3d.m_joint6;
                            }
                            if (src_idx == 7) {
                                window3d.m_joint_result_src = window3d.m_joint7;
                            }
                            if (src_idx == 8) {
                                window3d.m_joint_result_src = window3d.m_joint8;
                            }

                            vector<int> matching_idx;
                            vector<int> matching_result;
                            vector<int> matching_value;

                            for (int m = 0; m < window3d.m_joint_result_ref.size(); m++) {
                                matching_idx.push_back(window3d.m_joint_result_ref[m].num);
                            }

                            for (int m = 0; m < window3d.m_joint_result_src.size(); m++) {
                                for (int l = 0; l < matching_idx.size(); l++) {
                                    if (matching_idx[l] == window3d.m_joint_result_src[m].num) {
                                        matching_result.push_back(window3d.m_joint_result_src[m].num);
                                    }
                                }
                            }

                            for (int m = 0; m < window3d.m_joint_result_ref.size(); m++) {
                                for (int l = 0; l < matching_result.size(); l++) {
                                    if (matching_result[l] == window3d.m_joint_result_ref[m].num) {
                                        matching_value.push_back(window3d.m_joint_result_ref[m].num);
                                    }
                                }
                            }

                            struct joint_position {
                                int num;
                                float x;
                                float y;
                                float z;
                            };

                            vector<joint_position> ref_joint;
                            vector<joint_position> src_joint;


                            for (int m = 0; m < window3d.m_joint_result_ref.size(); m++) {
                                for (int l = 0; l < matching_value.size(); l++) {
                                    if (matching_value[l] == window3d.m_joint_result_ref[m].num) {
                                    
                                        if (loss_value[ref_idx - 1] == 0) {
                                            ref_joint.push_back({ window3d.m_joint_result_ref[m].num, window3d.m_joint_result_ref[m].x, window3d.m_joint_result_ref[m].y, window3d.m_joint_result_ref[m].z });
                                            printf("ref_cam : %d, num : %d, p : %f, %f, %f\n", ref_idx, window3d.m_joint_result_ref[m].num, window3d.m_joint_result_ref[m].x, window3d.m_joint_result_ref[m].y, window3d.m_joint_result_ref[m].z);
                                        }
                                        else {
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

                                            int num = ref_idx - 1;

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

                                            float px = window3d.m_joint_result_ref[m].x;
                                            float py = window3d.m_joint_result_ref[m].y;
                                            float pz = window3d.m_joint_result_ref[m].z;

                                            float ax = x_r1 * px + y_r1 * py + z_r1 * pz + (x_t1 / 1000);
                                            float ay = x_r2 * px + y_r2 * py + z_r2 * pz + (y_t1 / 1000);
                                            float az = x_r3 * px + y_r3 * py + z_r3 * pz + (z_t1 / 1000);

                                            ref_joint.push_back({ window3d.m_joint_result_ref[m].num, ax, ay, az });
                                            printf("ref_cam : %d, num : %d, p : %f, %f, %f\n", ref_idx, window3d.m_joint_result_ref[m].num, ax, ay, az);

                                            opti_count++;
                                        }

                                    }
                                }
                            }

                            for (int m = 0; m < window3d.m_joint_result_src.size(); m++) {
                                for (int l = 0; l < matching_value.size(); l++) {
                                    if (matching_value[l] == window3d.m_joint_result_src[m].num) {
                                        src_joint.push_back({ window3d.m_joint_result_src[m].num, window3d.m_joint_result_src[m].x, window3d.m_joint_result_src[m].y, window3d.m_joint_result_src[m].z });
                                        printf("src_cam : %d, num : %d, p : %f, %f, %f\n", src_idx, window3d.m_joint_result_src[m].num, window3d.m_joint_result_src[m].x, window3d.m_joint_result_src[m].y, window3d.m_joint_result_src[m].z);
                                    }
                                }
                            }



                            /* ----------------------------------------- */
                            /*                   Optimize                */
                            /* ----------------------------------------- */
                            
                            
                            cv::Mat R_x;
                            cv::Mat R_y;
                            cv::Mat R_z;
                            cv::Mat parameter_1;
                            
                            double learning_rate = 0.09;
                            double res_sum = 99;
                            double pre_res = 9999;
                            double res;
                            
                            double pitch = 0;
                            double yaw = 0;
                            double roll = 0;
                            double T[3] = { 0, 0, 0 };
                            
                            cv::Mat Co;
                            int f = 0;
                            
                            float defalt_ro[9] = { 1,0,0,
                                                   0,1,0,
                                                   0,0,1 };
                            
                            cv::Mat rotation(3, 3, CV_64FC1, defalt_ro);
                            cv::Mat translataion(3, 1, CV_64FC1, cv::Scalar(0));
                            
                            update_is = 0;
                            
                            cout << "--------------" << endl;
                            cout << "  Optimaize   " << endl;
                            cout << "--------------" << endl;
                            
                            while ((pre_res > res_sum)) {
                            
                                pre_res = res_sum;
                                cv::Mat parameter = (cv::Mat_<double>(1, 6) <<
                                    roll, pitch, yaw, T[0], T[1], T[2]);
                            
                            
                                memcpy(translataion.data, T, 3 * sizeof(double));
                            
                                // Calculate rotation about x axis
                            
                                R_x = (cv::Mat_<double>(3, 3) <<
                                    1, 0, 0,
                                    0, cos(pitch), -sin(pitch),
                                    0, sin(pitch), cos(pitch)
                                    );
                            
                                cv::Mat R_x_ = (cv::Mat_<double>(3, 3) <<
                                    0, 0, 0,
                                    0, -sin(pitch), -cos(pitch),
                                    0, cos(pitch), -sin(pitch)
                                    );
                            
                                // Calculate rotation about y axis
                                R_y = (cv::Mat_<double>(3, 3) <<
                                    cos(yaw), 0, sin(yaw),
                                    0, 1, 0,
                                    -sin(yaw), 0, cos(yaw)
                                    );
                            
                                cv::Mat R_y_ = (cv::Mat_<double>(3, 3) <<
                                    -sin(yaw), 0, cos(yaw),
                                    0, 0, 0,
                                    -cos(yaw), 0, -sin(yaw)
                                    );
                            
                                // Calculate rotation about z axis
                                R_z = (cv::Mat_<double>(3, 3) <<
                                    cos(roll), -sin(roll), 0,
                                    sin(roll), cos(roll), 0,
                                    0, 0, 1);
                            
                                cv::Mat R_z_ = (cv::Mat_<double>(3, 3) <<
                                    -sin(roll), -cos(roll), 0,
                                    cos(roll), -sin(roll), 0,
                                    0, 0, 0);
                            
                                cv::Mat dc_dp(1, 6, CV_64FC1, cv::Scalar(0));
                                cv::Mat dcost(3, 1, CV_64FC1, cv::Scalar(0));
                            
                                rotation = R_z * R_x * R_y;
                            
                                cv::Mat x_r(3, 1, CV_64FC1, cv::Scalar(0));
                                cv::Mat x_i(3, 1, CV_64FC1, cv::Scalar(0));
                                cv::Mat x_i_(3, 1, CV_64FC1, cv::Scalar(0));
                                cv::Mat Error;
                            
                                res_sum = 0.0;
                            
                                for (int i = 0; i < ref_joint.size(); i++)
                                {
                                    res = 0.0;
                                    x_r = cv::Scalar(0);
                                    x_i = cv::Scalar(0);
                                    x_i_ = cv::Scalar(0);
                                    Error = cv::Scalar(0);
                            
                                    x_r.at<double>(0, 0) = (double)ref_joint[i].x;
                                    x_r.at<double>(1, 0) = (double)ref_joint[i].y;
                                    x_r.at<double>(2, 0) = (double)ref_joint[i].z;
                            
                                    x_i.at<double>(0, 0) = (double)src_joint[i].x;
                                    x_i.at<double>(1, 0) = (double)src_joint[i].y;
                                    x_i.at<double>(2, 0) = (double)src_joint[i].z;
                            
                                    x_i_ = rotation * x_i + translataion;
                            
                                    x_i_ = x_r - x_i_;
                            
                                    Error = x_i_.t() * x_i_;
                                    res = Error.at<double>(0, 0);
                                    res_sum += res;
                            
                                    dcost = cv::Scalar(0);
                                    dcost = -((double)2 * (x_i_));
                            
                                    cv::Mat eq_rz = dcost.t() * R_z_ * R_x * R_y * x_i;
                                    cv::Mat eq_rx = dcost.t() * R_z * R_x_ * R_y * x_i;
                                    cv::Mat eq_ry = dcost.t() * R_z * R_x * R_y_ * x_i;
                            
                                    cv::Mat eq_tx = dcost.t() * (cv::Mat_<double>(3, 1) <<
                                        1, 0, 0);
                                    cv::Mat eq_ty = dcost.t() * (cv::Mat_<double>(3, 1) <<
                                        0, 1, 0);
                                    cv::Mat eq_tz = dcost.t() * (cv::Mat_<double>(3, 1) <<
                                        0, 0, 1);
                            
                                    dc_dp.at<double>(0, 0) += eq_rz.at<double>(0, 0);
                                    dc_dp.at<double>(0, 1) += eq_rx.at<double>(0, 0);
                                    dc_dp.at<double>(0, 2) += eq_ry.at<double>(0, 0);
                                    dc_dp.at<double>(0, 3) += eq_tx.at<double>(0, 0);
                                    dc_dp.at<double>(0, 4) += eq_ty.at<double>(0, 0);
                                    dc_dp.at<double>(0, 5) += eq_tz.at<double>(0, 0);
                            
                                }
                            
                                parameter_1 = cv::Scalar(0);
                                parameter_1 = (learning_rate / (double)ref_joint.size()) * dc_dp;
                                parameter_1 = parameter - parameter_1;
                            
                                if (f == 0 | f == 10 || f == 20 || f == 30 || f == 50 || f == 70 || f == 100 || f == 250 || f == 500 || f == 1000 || f == 2000 || f == 3000 || f == 5000 || f == 7000 || f == 9000) {
                                    printf("res_sum : %lf , 반복횟수 : %d, camera : %d\n", res_sum, f, src_idx);
                                }
                            
                                if (fabs(pre_res - res_sum) < 0.0000000001) {
                                    T[0] *= 1000.0;
                                    T[1] *= 1000.0;
                                    T[2] *= 1000.0;
                                    //Sleep(100000);
                                    memcpy(translataion.data, T, 3 * sizeof(double));
                                    // Calculate rotation about x axis
                                    R_x = (cv::Mat_<double>(3, 3) <<
                                        1, 0, 0,
                                        0, cos(pitch), -sin(pitch),
                                        0, sin(pitch), cos(pitch)
                                        );
                            
                                    // Calculate rotation about y axis
                                    R_y = (cv::Mat_<double>(3, 3) <<
                                        cos(yaw), 0, sin(yaw),
                                        0, 1, 0,
                                        -sin(yaw), 0, cos(yaw)
                                        );
                            
                                    // Calculate rotation about z axis
                                    R_z = (cv::Mat_<double>(3, 3) <<
                                        cos(roll), -sin(roll), 0,
                                        sin(roll), cos(roll), 0,
                                        0, 0, 1);
                            
                                    rotation = R_z * R_x * R_y;
                            
                                    if (loss_value[src_idx - 1] > res_sum && res_sum!=0) {
                                        update_is = 1;
                                    }
                                    else {
                                        update_is = 0;
                                    }
                                    printf("loss 비교 : %lf, %lf\n", loss_value[src_idx - 1], res_sum);
                                    printf("res_sum : %lf ,pre_sum : %lf  반복횟수 : %d,  learning rate : %.3f, camera : %d\n", res_sum, pre_res, f, learning_rate, src_idx);
                            
                                    if (update_is == 1) {
                                        loss_value[src_idx - 1] = res_sum;
                                    }
                                    break;
                                }
                            
                                roll = parameter_1.at<double>(0, 0);
                                pitch = parameter_1.at<double>(0, 1);
                                yaw = parameter_1.at<double>(0, 2);
                                T[0] = parameter_1.at<double>(0, 3);
                                T[1] = parameter_1.at<double>(0, 4);
                                T[2] = parameter_1.at<double>(0, 5);
                            
                                f++;
                            
                                if (((fabs(pre_res - res_sum) < 0.0001)) && (learning_rate > 0.0001)) {
                                    learning_rate = learning_rate - 0.000001;
                            
                                }
                            
                            }
                            
                            cout << "--------------" << endl;
                            cout << "   R_Matrix   " << endl;
                            cout << "--------------" << endl;
                            cout << rotation << endl;
                            cout << "--------------" << endl;
                            cout << "   t_Matrix   " << endl;
                            cout << "--------------" << endl;
                            cout << translataion << endl;
                            cout << "--------------" << endl;
                            
                            if (update_is == 1) {
                                
                                //cv::imwrite("cam1_" + to_string(iteration) + ".bmp", depth_map_clone);
                                
                                cv::imwrite("output/" + to_string(iteration) + "_cam1_depth" + ".bmp", depth_map_clone);
                                cv::imwrite("output/" + to_string(iteration) + "_cam2_depth" + ".bmp", depth_map_clone2);
                                cv::imwrite("output/" + to_string(iteration) + "_cam3_depth" + ".bmp", depth_map_clone3);
                                cv::imwrite("output/" + to_string(iteration) + "_cam4_depth" + ".bmp", depth_map_clone4);
                                cv::imwrite("output/" + to_string(iteration) + "_cam5_depth" + ".bmp", depth_map_clone5);
                                cv::imwrite("output/" + to_string(iteration) + "_cam6_depth" + ".bmp", depth_map_clone6);
                                cv::imwrite("output/" + to_string(iteration) + "_cam7_depth" + ".bmp", depth_map_clone7);
                                cv::imwrite("output/" + to_string(iteration) + "_cam8_depth" + ".bmp", depth_map_clone8);
                                           
                                cv::imwrite("output/" + to_string(iteration) + "_cam1_color" + ".bmp", color_map_clone);
                                cv::imwrite("output/" + to_string(iteration) + "_cam2_color" + ".bmp", color_map_clone2);
                                cv::imwrite("output/" + to_string(iteration) + "_cam3_color" + ".bmp", color_map_clone3);
                                cv::imwrite("output/" + to_string(iteration) + "_cam4_color" + ".bmp", color_map_clone4);
                                cv::imwrite("output/" + to_string(iteration) + "_cam5_color" + ".bmp", color_map_clone5);
                                cv::imwrite("output/" + to_string(iteration) + "_cam6_color" + ".bmp", color_map_clone6);
                                cv::imwrite("output/" + to_string(iteration) + "_cam7_color" + ".bmp", color_map_clone7);
                                cv::imwrite("output/" + to_string(iteration) + "_cam8_color" + ".bmp", color_map_clone8);
                                
                                //window3d.m_joint_result8

                                
                                std::string file_name;
                                std::string file_name2;
                                int point_count = 0;
                                for (int num = 0; num < 8; num++) {
                                    std::stringstream ss;
                                    std::stringstream ss2;

                                    int point_count = 0;

                                    int bone_count = 0;
                                    int bone_count1 = 0;
                                    int bone_count2 = 0;
                                    int bone_count3 = 0;
                                    int bone_count4 = 0;
                                    int bone_count5 = 0;
                                    int bone_count6 = 0;
                                    int bone_count7 = 0;
                                    int bone_count8 = 0;

                                    if (num == 0) {
                                        file_name = to_string(iteration) + "_"+to_string(num + 1)+"_joint.ply";
                                        file_name2 = to_string(iteration) + "_" + to_string(num + 1) + "_matrix.txt";
                                        point_count = window3d.m_joint.size();
                                        for(int m=0; m < window3d.m_joint.size(); m++){
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

                                            float px = window3d.m_joint[m].x;
                                            float py = window3d.m_joint[m].y;
                                            float pz = window3d.m_joint[m].z;
                                            int joint_num = window3d.m_joint[m].num;

                                            float ax = x_r1 * px + y_r1 * py + z_r1 * pz + (x_t1 / 1000);
                                            float ay = x_r2 * px + y_r2 * py + z_r2 * pz + (y_t1 / 1000);
                                            float az = x_r3 * px + y_r3 * py + z_r3 * pz + (z_t1 / 1000);

                                            ss << ax << " " << ay << " " << az << " " << joint_num << std::endl;
                                        }
                                        for (size_t boneIdx = 0; boneIdx < g_boneList.size(); boneIdx++)
                                        {
                                            k4abt_joint_id_t joint1 = g_boneList[boneIdx].first;
                                            k4abt_joint_id_t joint2 = g_boneList[boneIdx].second;


                                            if (boneIdx != 10 && boneIdx != 16 && boneIdx != 17 && boneIdx != 23 && boneIdx != 29 && boneIdx != 30 && boneIdx != 11 && boneIdx != 24 && boneIdx != 9 && boneIdx != 22) {
                                                int aaa = 0;
                                                int bbb = 0;
                                                int index = 0;
                                                for (int iii = 0; iii < window3d.m_joint.size(); iii++) {
                                                    int chk = isnan(float(window3d.m_joint[iii].x));


                                                    if (chk != 0) {
                                                        continue;
                                                    }


                                                    int chk2 = isnan(float(window3d.m_joint[iii].x));

                                                    if (chk2 != 0) {
                                                        continue;
                                                    }

                                                    if (joint1 == window3d.m_joint[iii].num) {
                                                        aaa = index;
                                                    }
                                                    if (joint2 == window3d.m_joint[iii].num) {
                                                        bbb = index;
                                                    }
                                                    index++;
                                                }

                                                if (joint2 == 8 || joint2 == 15) {
                                                    continue;
                                                }

                                                if (aaa != bbb && bbb != 0) {
                                                    ss << aaa << " " << bbb << std::endl;
                                                    bone_count++;
                                                }
                                                else {
                                                    if (joint1 == 1 && bbb == 0) {
                                                        ss << aaa << " " << bbb << std::endl;
                                                        bone_count++;
                                                    }
                                                }
                                            }
                                        }



                                        int k = num * 15;

                                        float x_r1 = extrinsic[num + k];
                                        float y_r1 = extrinsic[num + 1 + k];
                                        float z_r1 = extrinsic[num + 2 + k];
                                        float x_t1 = extrinsic[num + 3 + k];

                                        float x_r2 = extrinsic[num + 4 + k];
                                        float y_r2 = extrinsic[num + 5 + k];
                                        float z_r2 = extrinsic[num + 6 + k];
                                        float y_t1 = extrinsic[num + 7 + k];

                                        float x_r3 = extrinsic[num + 8 + k];
                                        float y_r3 = extrinsic[num + 9 + k];
                                        float z_r3 = extrinsic[num + 10 + k];
                                        float z_t1 = extrinsic[num + 11 + k];

                                        ss2 << x_r1 << " " << y_r1 << " " << z_r1 << " " << x_t1 << std::endl;
                                        ss2 << x_r2 << " " << y_r2 << " " << z_r2 << " " << y_t1 << std::endl;
                                        ss2 << x_r3 << " " << y_r3 << " " << z_r3 << " " << z_t1 << std::endl;
                                        ss2 << 0 << " " << 0 << " " << 0 << " " << 1 << std::endl;

                                    }
                                    if (num == 1) {
                                        file_name = to_string(iteration) + "_" + to_string(num + 1) + "_joint.ply";
                                        file_name2 = to_string(iteration) + "_" + to_string(num + 1) + "_matrix.txt";
                                        point_count = window3d.m_joint2.size();
                                        for (int m = 0; m < window3d.m_joint2.size(); m++) {
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

                                            float px = window3d.m_joint2[m].x;
                                            float py = window3d.m_joint2[m].y;
                                            float pz = window3d.m_joint2[m].z;
                                            int joint_num = window3d.m_joint2[m].num;

                                            float ax = x_r1 * px + y_r1 * py + z_r1 * pz + (x_t1 / 1000);
                                            float ay = x_r2 * px + y_r2 * py + z_r2 * pz + (y_t1 / 1000);
                                            float az = x_r3 * px + y_r3 * py + z_r3 * pz + (z_t1 / 1000);

                                            ss << ax << " " << ay << " " << az << " " << joint_num << std::endl;
                                        }
                                        for (size_t boneIdx = 0; boneIdx < g_boneList.size(); boneIdx++)
                                        {
                                            k4abt_joint_id_t joint1 = g_boneList[boneIdx].first;
                                            k4abt_joint_id_t joint2 = g_boneList[boneIdx].second;

                                            if (boneIdx != 10 && boneIdx != 16 && boneIdx != 17 && boneIdx != 23 && boneIdx != 29 && boneIdx != 30 && boneIdx != 11 && boneIdx != 24 && boneIdx != 9 && boneIdx != 22) {
                                                int aaa = 0;
                                                int bbb = 0;
                                                int index = 0;
                                                for (int iii = 0; iii < window3d.m_joint2.size(); iii++) {
                                                    int chk = isnan(float(window3d.m_joint2[iii].x));


                                                    if (chk != 0) {
                                                        continue;
                                                    }


                                                    int chk2 = isnan(float(window3d.m_joint2[iii].x));

                                                    if (chk2 != 0) {
                                                        continue;
                                                    }

                                                    if (joint1 == window3d.m_joint2[iii].num) {
                                                        aaa = index;
                                                    }
                                                    if (joint2 == window3d.m_joint2[iii].num) {
                                                        bbb = index;
                                                    }
                                                    index++;
                                                }

                                                if (joint2 == 8 || joint2 == 15) {
                                                    continue;
                                                }

                                                if (aaa != bbb && bbb != 0) {
                                                    ss << aaa << " " << bbb << std::endl;
                                                    bone_count1++;
                                                }
                                                else {
                                                    if (joint1 == 1 && bbb == 0) {
                                                        ss << aaa << " " << bbb << std::endl;
                                                        bone_count1++;
                                                    }
                                                }
                                            }
                                        }

                                        int k = num * 15;

                                        float x_r1 = extrinsic[num + k];
                                        float y_r1 = extrinsic[num + 1 + k];
                                        float z_r1 = extrinsic[num + 2 + k];
                                        float x_t1 = extrinsic[num + 3 + k];

                                        float x_r2 = extrinsic[num + 4 + k];
                                        float y_r2 = extrinsic[num + 5 + k];
                                        float z_r2 = extrinsic[num + 6 + k];
                                        float y_t1 = extrinsic[num + 7 + k];

                                        float x_r3 = extrinsic[num + 8 + k];
                                        float y_r3 = extrinsic[num + 9 + k];
                                        float z_r3 = extrinsic[num + 10 + k];
                                        float z_t1 = extrinsic[num + 11 + k];

                                        ss2 << x_r1 << " " << y_r1 << " " << z_r1 << " " << x_t1 << std::endl;
                                        ss2 << x_r2 << " " << y_r2 << " " << z_r2 << " " << y_t1 << std::endl;
                                        ss2 << x_r3 << " " << y_r3 << " " << z_r3 << " " << z_t1 << std::endl;
                                        ss2 << 0 << " " << 0 << " " << 0 << " " << 1 << std::endl;
                                    }
                                    if (num == 2) {
                                        file_name = to_string(iteration) + "_" + to_string(num + 1) + "_joint.ply";
                                        file_name2 = to_string(iteration) + "_" + to_string(num + 1) + "_matrix.txt";
                                        point_count = window3d.m_joint3.size();
                                        for (int m = 0; m < window3d.m_joint3.size(); m++) {
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

                                            float px = window3d.m_joint3[m].x;
                                            float py = window3d.m_joint3[m].y;
                                            float pz = window3d.m_joint3[m].z;
                                            int joint_num = window3d.m_joint3[m].num;

                                            float ax = x_r1 * px + y_r1 * py + z_r1 * pz + (x_t1 / 1000);
                                            float ay = x_r2 * px + y_r2 * py + z_r2 * pz + (y_t1 / 1000);
                                            float az = x_r3 * px + y_r3 * py + z_r3 * pz + (z_t1 / 1000);

                                            ss << ax << " " << ay << " " << az << " " << joint_num << std::endl;
                                        }
                                        for (size_t boneIdx = 0; boneIdx < g_boneList.size(); boneIdx++)
                                        {
                                            k4abt_joint_id_t joint1 = g_boneList[boneIdx].first;
                                            k4abt_joint_id_t joint2 = g_boneList[boneIdx].second;

                                            if (boneIdx != 10 && boneIdx != 16 && boneIdx != 17 && boneIdx != 23 && boneIdx != 29 && boneIdx != 30 && boneIdx != 11 && boneIdx != 24 && boneIdx != 9 && boneIdx != 22) {
                                                int aaa = 0;
                                                int bbb = 0;
                                                int index = 0;
                                                for (int iii = 0; iii < window3d.m_joint3.size(); iii++) {
                                                    int chk = isnan(float(window3d.m_joint3[iii].x));


                                                    if (chk != 0) {
                                                        continue;
                                                    }


                                                    int chk2 = isnan(float(window3d.m_joint3[iii].x));

                                                    if (chk2 != 0) {
                                                        continue;
                                                    }

                                                    if (joint1 == window3d.m_joint3[iii].num) {
                                                        aaa = index;
                                                    }
                                                    if (joint2 == window3d.m_joint3[iii].num) {
                                                        bbb = index;
                                                    }
                                                    index++;
                                                }

                                                if (joint2 == 8 || joint2 == 15) {
                                                    continue;
                                                }

                                                if (aaa != bbb && bbb != 0) {
                                                    ss << aaa << " " << bbb << std::endl;
                                                    bone_count2++;
                                                }
                                                else {
                                                    if (joint1 == 1 && bbb == 0) {
                                                        ss << aaa << " " << bbb << std::endl;
                                                        bone_count2++;
                                                    }
                                                }
                                            }
                                        }

                                        int k = num * 15;

                                        float x_r1 = extrinsic[num + k];
                                        float y_r1 = extrinsic[num + 1 + k];
                                        float z_r1 = extrinsic[num + 2 + k];
                                        float x_t1 = extrinsic[num + 3 + k];

                                        float x_r2 = extrinsic[num + 4 + k];
                                        float y_r2 = extrinsic[num + 5 + k];
                                        float z_r2 = extrinsic[num + 6 + k];
                                        float y_t1 = extrinsic[num + 7 + k];

                                        float x_r3 = extrinsic[num + 8 + k];
                                        float y_r3 = extrinsic[num + 9 + k];
                                        float z_r3 = extrinsic[num + 10 + k];
                                        float z_t1 = extrinsic[num + 11 + k];

                                        ss2 << x_r1 << " " << y_r1 << " " << z_r1 << " " << x_t1 << std::endl;
                                        ss2 << x_r2 << " " << y_r2 << " " << z_r2 << " " << y_t1 << std::endl;
                                        ss2 << x_r3 << " " << y_r3 << " " << z_r3 << " " << z_t1 << std::endl;
                                        ss2 << 0 << " " << 0 << " " << 0 << " " << 1 << std::endl;
                                    }
                                    if (num == 3) {
                                        file_name = to_string(iteration) + "_" + to_string(num + 1) + "_joint.ply";
                                        file_name2 = to_string(iteration) + "_" + to_string(num + 1) + "_matrix.txt";
                                        point_count = window3d.m_joint4.size();
                                        for (int m = 0; m < window3d.m_joint4.size(); m++) {
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

                                            float px = window3d.m_joint4[m].x;
                                            float py = window3d.m_joint4[m].y;
                                            float pz = window3d.m_joint4[m].z;
                                            int joint_num = window3d.m_joint4[m].num;

                                            float ax = x_r1 * px + y_r1 * py + z_r1 * pz + (x_t1 / 1000);
                                            float ay = x_r2 * px + y_r2 * py + z_r2 * pz + (y_t1 / 1000);
                                            float az = x_r3 * px + y_r3 * py + z_r3 * pz + (z_t1 / 1000);

                                            ss << ax << " " << ay << " " << az << " " << joint_num << std::endl;
                                        }
                                        for (size_t boneIdx = 0; boneIdx < g_boneList.size(); boneIdx++)
                                        {
                                            k4abt_joint_id_t joint1 = g_boneList[boneIdx].first;
                                            k4abt_joint_id_t joint2 = g_boneList[boneIdx].second;

                                            if (boneIdx != 10 && boneIdx != 16 && boneIdx != 17 && boneIdx != 23 && boneIdx != 29 && boneIdx != 30 && boneIdx != 11 && boneIdx != 24 && boneIdx != 9 && boneIdx != 22) {
                                                int aaa = 0;
                                                int bbb = 0;
                                                int index = 0;
                                                for (int iii = 0; iii < window3d.m_joint4.size(); iii++) {
                                                    int chk = isnan(float(window3d.m_joint4[iii].x));


                                                    if (chk != 0) {
                                                        continue;
                                                    }


                                                    int chk2 = isnan(float(window3d.m_joint4[iii].x));

                                                    if (chk2 != 0) {
                                                        continue;
                                                    }

                                                    if (joint1 == window3d.m_joint4[iii].num) {
                                                        aaa = index;
                                                    }
                                                    if (joint2 == window3d.m_joint4[iii].num) {
                                                        bbb = index;
                                                    }
                                                    index++;
                                                }

                                                if (joint2 == 8 || joint2 == 15) {
                                                    continue;
                                                }

                                                if (aaa != bbb && bbb != 0) {
                                                    ss << aaa << " " << bbb << std::endl;
                                                    bone_count3++;
                                                }
                                                else {
                                                    if (joint1 == 1 && bbb == 0) {
                                                        ss << aaa << " " << bbb << std::endl;
                                                        bone_count3++;
                                                    }
                                                }
                                            }
                                        }

                                        int k = num * 15;

                                        float x_r1 = extrinsic[num + k];
                                        float y_r1 = extrinsic[num + 1 + k];
                                        float z_r1 = extrinsic[num + 2 + k];
                                        float x_t1 = extrinsic[num + 3 + k];

                                        float x_r2 = extrinsic[num + 4 + k];
                                        float y_r2 = extrinsic[num + 5 + k];
                                        float z_r2 = extrinsic[num + 6 + k];
                                        float y_t1 = extrinsic[num + 7 + k];

                                        float x_r3 = extrinsic[num + 8 + k];
                                        float y_r3 = extrinsic[num + 9 + k];
                                        float z_r3 = extrinsic[num + 10 + k];
                                        float z_t1 = extrinsic[num + 11 + k];

                                        ss2 << x_r1 << " " << y_r1 << " " << z_r1 << " " << x_t1 << std::endl;
                                        ss2 << x_r2 << " " << y_r2 << " " << z_r2 << " " << y_t1 << std::endl;
                                        ss2 << x_r3 << " " << y_r3 << " " << z_r3 << " " << z_t1 << std::endl;
                                        ss2 << 0 << " " << 0 << " " << 0 << " " << 1 << std::endl;
                                    }
                                    if (num == 4) {
                                        file_name = to_string(iteration) + "_" + to_string(num + 1) + "_joint.ply";
                                        file_name2 = to_string(iteration) + "_" + to_string(num + 1) + "_matrix.txt";
                                        point_count = window3d.m_joint5.size();
                                        for (int m = 0; m < window3d.m_joint5.size(); m++) {
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

                                            float px = window3d.m_joint5[m].x;
                                            float py = window3d.m_joint5[m].y;
                                            float pz = window3d.m_joint5[m].z;
                                            int joint_num = window3d.m_joint5[m].num;

                                            float ax = x_r1 * px + y_r1 * py + z_r1 * pz + (x_t1 / 1000);
                                            float ay = x_r2 * px + y_r2 * py + z_r2 * pz + (y_t1 / 1000);
                                            float az = x_r3 * px + y_r3 * py + z_r3 * pz + (z_t1 / 1000);

                                            ss << ax << " " << ay << " " << az << " " << joint_num << std::endl;
                                        }
                                        for (size_t boneIdx = 0; boneIdx < g_boneList.size(); boneIdx++)
                                        {
                                            k4abt_joint_id_t joint1 = g_boneList[boneIdx].first;
                                            k4abt_joint_id_t joint2 = g_boneList[boneIdx].second;

                                            if (boneIdx != 10 && boneIdx != 16 && boneIdx != 17 && boneIdx != 23 && boneIdx != 29 && boneIdx != 30 && boneIdx != 11 && boneIdx != 24 && boneIdx != 9 && boneIdx != 22) {
                                                int aaa = 0;
                                                int bbb = 0;
                                                int index = 0;
                                                for (int iii = 0; iii < window3d.m_joint5.size(); iii++) {
                                                    int chk = isnan(float(window3d.m_joint5[iii].x));


                                                    if (chk != 0) {
                                                        continue;
                                                    }


                                                    int chk2 = isnan(float(window3d.m_joint5[iii].x));

                                                    if (chk2 != 0) {
                                                        continue;
                                                    }

                                                    if (joint1 == window3d.m_joint5[iii].num) {
                                                        aaa = index;
                                                    }
                                                    if (joint2 == window3d.m_joint5[iii].num) {
                                                        bbb = index;
                                                    }
                                                    index++;
                                                }

                                                if (joint2 == 8 || joint2 == 15) {
                                                    continue;
                                                }

                                                if (aaa != bbb && bbb != 0) {
                                                    ss << aaa << " " << bbb << std::endl;
                                                    bone_count4++;
                                                }
                                                else {
                                                    if (joint1 == 1 && bbb == 0) {
                                                        ss << aaa << " " << bbb << std::endl;
                                                        bone_count4++;
                                                    }
                                                }
                                            }
                                        }

                                        int k = num * 15;

                                        float x_r1 = extrinsic[num + k];
                                        float y_r1 = extrinsic[num + 1 + k];
                                        float z_r1 = extrinsic[num + 2 + k];
                                        float x_t1 = extrinsic[num + 3 + k];

                                        float x_r2 = extrinsic[num + 4 + k];
                                        float y_r2 = extrinsic[num + 5 + k];
                                        float z_r2 = extrinsic[num + 6 + k];
                                        float y_t1 = extrinsic[num + 7 + k];

                                        float x_r3 = extrinsic[num + 8 + k];
                                        float y_r3 = extrinsic[num + 9 + k];
                                        float z_r3 = extrinsic[num + 10 + k];
                                        float z_t1 = extrinsic[num + 11 + k];

                                        ss2 << x_r1 << " " << y_r1 << " " << z_r1 << " " << x_t1 << std::endl;
                                        ss2 << x_r2 << " " << y_r2 << " " << z_r2 << " " << y_t1 << std::endl;
                                        ss2 << x_r3 << " " << y_r3 << " " << z_r3 << " " << z_t1 << std::endl;
                                        ss2 << 0 << " " << 0 << " " << 0 << " " << 1 << std::endl;
                                    }
                                    if (num == 5) {
                                        file_name = to_string(iteration) + "_" + to_string(num + 1) + "_joint.ply";
                                        file_name2 = to_string(iteration) + "_" + to_string(num + 1) + "_matrix.txt";
                                        point_count = window3d.m_joint6.size();
                                        for (int m = 0; m < window3d.m_joint6.size(); m++) {
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

                                            float px = window3d.m_joint6[m].x;
                                            float py = window3d.m_joint6[m].y;
                                            float pz = window3d.m_joint6[m].z;
                                            int joint_num = window3d.m_joint6[m].num;

                                            float ax = x_r1 * px + y_r1 * py + z_r1 * pz + (x_t1 / 1000);
                                            float ay = x_r2 * px + y_r2 * py + z_r2 * pz + (y_t1 / 1000);
                                            float az = x_r3 * px + y_r3 * py + z_r3 * pz + (z_t1 / 1000);

                                            ss << ax << " " << ay << " " << az << " " << joint_num << std::endl;
                                        }
                                        for (size_t boneIdx = 0; boneIdx < g_boneList.size(); boneIdx++)
                                        {
                                            k4abt_joint_id_t joint1 = g_boneList[boneIdx].first;
                                            k4abt_joint_id_t joint2 = g_boneList[boneIdx].second;

                                            if (boneIdx != 10 && boneIdx != 16 && boneIdx != 17 && boneIdx != 23 && boneIdx != 29 && boneIdx != 30 && boneIdx != 11 && boneIdx != 24 && boneIdx != 9 && boneIdx != 22) {
                                                int aaa = 0;
                                                int bbb = 0;
                                                int index = 0;
                                                for (int iii = 0; iii < window3d.m_joint6.size(); iii++) {
                                                    int chk = isnan(float(window3d.m_joint6[iii].x));


                                                    if (chk != 0) {
                                                        continue;
                                                    }


                                                    int chk2 = isnan(float(window3d.m_joint6[iii].x));

                                                    if (chk2 != 0) {
                                                        continue;
                                                    }

                                                    if (joint1 == window3d.m_joint6[iii].num) {
                                                        aaa = index;
                                                    }
                                                    if (joint2 == window3d.m_joint6[iii].num) {
                                                        bbb = index;
                                                    }
                                                    index++;
                                                }

                                                if (joint2 == 8 || joint2 == 15) {
                                                    continue;
                                                }

                                                if (aaa != bbb && bbb != 0) {
                                                    ss << aaa << " " << bbb << std::endl;
                                                    bone_count5++;
                                                }
                                                else {
                                                    if (joint1 == 1 && bbb == 0) {
                                                        ss << aaa << " " << bbb << std::endl;
                                                        bone_count5++;
                                                    }
                                                }
                                            }
                                        }
                                        int k = num * 15;

                                        float x_r1 = extrinsic[num + k];
                                        float y_r1 = extrinsic[num + 1 + k];
                                        float z_r1 = extrinsic[num + 2 + k];
                                        float x_t1 = extrinsic[num + 3 + k];

                                        float x_r2 = extrinsic[num + 4 + k];
                                        float y_r2 = extrinsic[num + 5 + k];
                                        float z_r2 = extrinsic[num + 6 + k];
                                        float y_t1 = extrinsic[num + 7 + k];

                                        float x_r3 = extrinsic[num + 8 + k];
                                        float y_r3 = extrinsic[num + 9 + k];
                                        float z_r3 = extrinsic[num + 10 + k];
                                        float z_t1 = extrinsic[num + 11 + k];

                                        ss2 << x_r1 << " " << y_r1 << " " << z_r1 << " " << x_t1 << std::endl;
                                        ss2 << x_r2 << " " << y_r2 << " " << z_r2 << " " << y_t1 << std::endl;
                                        ss2 << x_r3 << " " << y_r3 << " " << z_r3 << " " << z_t1 << std::endl;
                                        ss2 << 0 << " " << 0 << " " << 0 << " " << 1 << std::endl;
                                    }
                                    if (num == 6) {
                                        file_name = to_string(iteration) + "_" + to_string(num + 1) + "_joint.ply";
                                        file_name2 = to_string(iteration) + "_" + to_string(num + 1) + "_matrix.txt";
                                        point_count = window3d.m_joint7.size();
                                        for (int m = 0; m < window3d.m_joint7.size(); m++) {
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

                                            float px = window3d.m_joint7[m].x;
                                            float py = window3d.m_joint7[m].y;
                                            float pz = window3d.m_joint7[m].z;
                                            int joint_num = window3d.m_joint7[m].num;

                                            float ax = x_r1 * px + y_r1 * py + z_r1 * pz + (x_t1 / 1000);
                                            float ay = x_r2 * px + y_r2 * py + z_r2 * pz + (y_t1 / 1000);
                                            float az = x_r3 * px + y_r3 * py + z_r3 * pz + (z_t1 / 1000);

                                            ss << ax << " " << ay << " " << az << " " << joint_num << std::endl;
                                        }
                                        for (size_t boneIdx = 0; boneIdx < g_boneList.size(); boneIdx++)
                                        {
                                            k4abt_joint_id_t joint1 = g_boneList[boneIdx].first;
                                            k4abt_joint_id_t joint2 = g_boneList[boneIdx].second;

                                            if (boneIdx != 10 && boneIdx != 16 && boneIdx != 17 && boneIdx != 23 && boneIdx != 29 && boneIdx != 30 && boneIdx != 11 && boneIdx != 24 && boneIdx != 9 && boneIdx != 22) {
                                                int aaa = 0;
                                                int bbb = 0;
                                                int index = 0;
                                                for (int iii = 0; iii < window3d.m_joint7.size(); iii++) {
                                                    int chk = isnan(float(window3d.m_joint7[iii].x));


                                                    if (chk != 0) {
                                                        continue;
                                                    }


                                                    int chk2 = isnan(float(window3d.m_joint7[iii].x));

                                                    if (chk2 != 0) {
                                                        continue;
                                                    }

                                                    if (joint1 == window3d.m_joint7[iii].num) {
                                                        aaa = index;
                                                    }
                                                    if (joint2 == window3d.m_joint7[iii].num) {
                                                        bbb = index;
                                                    }
                                                    index++;
                                                }

                                                if (joint2 == 8 || joint2 == 15) {
                                                    continue;
                                                }

                                                if (aaa != bbb && bbb != 0) {
                                                    ss << aaa << " " << bbb << std::endl;
                                                    bone_count6++;
                                                }
                                                else {
                                                    if (joint1 == 1 && bbb == 0) {
                                                        ss << aaa << " " << bbb << std::endl;
                                                        bone_count6++;
                                                    }
                                                }
                                            }
                                        }
                                        int k = num * 15;

                                        float x_r1 = extrinsic[num + k];
                                        float y_r1 = extrinsic[num + 1 + k];
                                        float z_r1 = extrinsic[num + 2 + k];
                                        float x_t1 = extrinsic[num + 3 + k];

                                        float x_r2 = extrinsic[num + 4 + k];
                                        float y_r2 = extrinsic[num + 5 + k];
                                        float z_r2 = extrinsic[num + 6 + k];
                                        float y_t1 = extrinsic[num + 7 + k];

                                        float x_r3 = extrinsic[num + 8 + k];
                                        float y_r3 = extrinsic[num + 9 + k];
                                        float z_r3 = extrinsic[num + 10 + k];
                                        float z_t1 = extrinsic[num + 11 + k];

                                        ss2 << x_r1 << " " << y_r1 << " " << z_r1 << " " << x_t1 << std::endl;
                                        ss2 << x_r2 << " " << y_r2 << " " << z_r2 << " " << y_t1 << std::endl;
                                        ss2 << x_r3 << " " << y_r3 << " " << z_r3 << " " << z_t1 << std::endl;
                                        ss2 << 0 << " " << 0 << " " << 0 << " " << 1 << std::endl;
                                    }
                                    if (num == 7) {
                                        file_name = to_string(iteration) + "_" + to_string(num + 1) + "_joint.ply";
                                        file_name2 = to_string(iteration) + "_" + to_string(num + 1) + "_matrix.txt";
                                        point_count = window3d.m_joint8.size();
                                        for (int m = 0; m < window3d.m_joint8.size(); m++) {
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

                                            float px = window3d.m_joint8[m].x;
                                            float py = window3d.m_joint8[m].y;
                                            float pz = window3d.m_joint8[m].z;
                                            int joint_num = window3d.m_joint8[m].num;

                                            float ax = x_r1 * px + y_r1 * py + z_r1 * pz + (x_t1 / 1000);
                                            float ay = x_r2 * px + y_r2 * py + z_r2 * pz + (y_t1 / 1000);
                                            float az = x_r3 * px + y_r3 * py + z_r3 * pz + (z_t1 / 1000);

                                            ss << ax << " " << ay << " " << az << " " << joint_num << std::endl;
                                        }
                                        for (size_t boneIdx = 0; boneIdx < g_boneList.size(); boneIdx++)
                                        {
                                            k4abt_joint_id_t joint1 = g_boneList[boneIdx].first;
                                            k4abt_joint_id_t joint2 = g_boneList[boneIdx].second;

                                            if (boneIdx != 10 && boneIdx != 16 && boneIdx != 17 && boneIdx != 23 && boneIdx != 29 && boneIdx != 30 && boneIdx != 11 && boneIdx != 24 && boneIdx != 9 && boneIdx != 22) {
                                                int aaa = 0;
                                                int bbb = 0;
                                                int index = 0;
                                                for (int iii = 0; iii < window3d.m_joint8.size(); iii++) {
                                                    int chk = isnan(float(window3d.m_joint8[iii].x));


                                                    if (chk != 0) {
                                                        continue;
                                                    }


                                                    int chk2 = isnan(float(window3d.m_joint8[iii].x));

                                                    if (chk2 != 0) {
                                                        continue;
                                                    }

                                                    if (joint1 == window3d.m_joint8[iii].num) {
                                                        aaa = index;
                                                    }
                                                    if (joint2 == window3d.m_joint8[iii].num) {
                                                        bbb = index;
                                                    }
                                                    index++;
                                                }

                                                if (joint2 == 8 || joint2 == 15) {
                                                    continue;
                                                }

                                                if (aaa != bbb && bbb != 0) {
                                                    ss << aaa << " " << bbb << std::endl;
                                                    bone_count7++;
                                                }
                                                else {
                                                    if (joint1 == 1 && bbb == 0) {
                                                        ss << aaa << " " << bbb << std::endl;
                                                        bone_count7++;
                                                    }
                                                }
                                            }
                                        }
                                        int k = num * 15;

                                        float x_r1 = extrinsic[num + k];
                                        float y_r1 = extrinsic[num + 1 + k];
                                        float z_r1 = extrinsic[num + 2 + k];
                                        float x_t1 = extrinsic[num + 3 + k];

                                        float x_r2 = extrinsic[num + 4 + k];
                                        float y_r2 = extrinsic[num + 5 + k];
                                        float z_r2 = extrinsic[num + 6 + k];
                                        float y_t1 = extrinsic[num + 7 + k];

                                        float x_r3 = extrinsic[num + 8 + k];
                                        float y_r3 = extrinsic[num + 9 + k];
                                        float z_r3 = extrinsic[num + 10 + k];
                                        float z_t1 = extrinsic[num + 11 + k];

                                        ss2 << x_r1 << " " << y_r1 << " " << z_r1 << " " << x_t1 << std::endl;
                                        ss2 << x_r2 << " " << y_r2 << " " << z_r2 << " " << y_t1 << std::endl;
                                        ss2 << x_r3 << " " << y_r3 << " " << z_r3 << " " << z_t1 << std::endl;
                                        ss2 << 0 << " " << 0 << " " << 0 << " " << 1 << std::endl;
                                    }


                                    if (num == 0) {
                                        bone_count = bone_count;
                                    }
                                    if (num == 1) {
                                        bone_count = bone_count1;
                                    }
                                    if (num == 2) {
                                        bone_count = bone_count2;
                                    }
                                    if (num == 3) {
                                        bone_count = bone_count3;
                                    }
                                    if (num == 4) {
                                        bone_count = bone_count4;
                                    }
                                    if (num == 5) {
                                        bone_count = bone_count5;
                                    }
                                    if (num == 6) {
                                        bone_count = bone_count6;
                                    }
                                    if (num == 7) {
                                        bone_count = bone_count7;
                                    }
                                    if (num == 8) {
                                        bone_count = bone_count8;
                                    }

                                    if (bone_count == 1) {
                                        bone_count = 0;
                                    }

                                    std::ofstream ofs(file_name.c_str());
                                    ofs << "ply" << std::endl;
                                    ofs << "format ascii 1.0" << std::endl;
                                    ofs << "element vertex"
                                        << " " << point_count << std::endl;
                                    ofs << "property float x" << std::endl;
                                    ofs << "property float y" << std::endl;
                                    ofs << "property float z" << std::endl;
                                    ofs << "property int num" << std::endl;
                                    ofs << "element edge"
                                        << " " << bone_count << std::endl;
                                    ofs << "property int vertex1" << std::endl;
                                    ofs << "property int vertex2" << std::endl;
                                    ofs << "end_header" << std::endl;
                                    ofs.close();

                                    std::ofstream ofs_text(file_name, std::ios::out | std::ios::app);
                                    ofs_text.write(ss.str().c_str(), (std::streamsize)ss.str().length());

                                    std::ofstream ofs_text2(file_name2, std::ios::out | std::ios::app);
                                    ofs_text2.write(ss2.str().c_str(), (std::streamsize)ss2.str().length());

                                }

                                
                                std::stringstream ss3;
                                for (int loss_size = 0; loss_size < loss_value.size(); loss_size++) {
                                    ss3 << iteration << " " << loss_size + 1 << " " << static_cast<float>(loss_value[loss_size]) << std::endl;
                                }                                
                                std::ofstream ofs_text("loss.txt", std::ios::out | std::ios::app);
                                ofs_text.write(ss3.str().c_str(), (std::streamsize)ss3.str().length());


                                int num = src_idx - 1;
                                int k = num * 15;
                                extrinsic[num + k] = rotation.at<double>(0, 0);
                                extrinsic[num + 1 + k] = rotation.at<double>(0, 1);
                                extrinsic[num + 2 + k] = rotation.at<double>(0, 2);
                                extrinsic[num + 3 + k] = translataion.at<double>(0, 0);
                            
                                extrinsic[num + 4 + k] = rotation.at<double>(1, 0);
                                extrinsic[num + 5 + k] = rotation.at<double>(1, 1);
                                extrinsic[num + 6 + k] = rotation.at<double>(1, 2);
                                extrinsic[num + 7 + k] = translataion.at<double>(0, 1);
                            
                                extrinsic[num + 8 + k] = rotation.at<double>(2, 0);
                                extrinsic[num + 9 + k] = rotation.at<double>(2, 1);
                                extrinsic[num + 10 + k] = rotation.at<double>(2, 2);
                                extrinsic[num + 11 + k] = translataion.at<double>(0, 2);
                            
                                printf("********** %d update!! ********** \n", num + 1);
                            }
                        }                       
                    }


                    printf("------------------------------------\n");
                    //Sleep(100000);
            }           

            k4a_image_release(depthImage);
            k4a_image_release(depthImage2);
            k4a_image_release(depthImage3);
            k4a_image_release(depthImage4);
            k4a_image_release(depthImage5);
            k4a_image_release(depthImage6);
            k4a_image_release(depthImage7);
            k4a_image_release(depthImage8);


            k4a_image_release(colorImage);
            k4a_image_release(colorImage2);
            k4a_image_release(colorImage3);
            k4a_image_release(colorImage4);
            k4a_image_release(colorImage5);
            k4a_image_release(colorImage6);
            k4a_image_release(colorImage7);
            k4a_image_release(colorImage8);

        }
        }


        // joint 확인용
        //for (int i = 0; i < window3d.m_joint.size(); i++) {
        //    printf("camera 1, joint %d, %f, %f, %f\n", window3d.m_joint[i].num, window3d.m_joint[i].x, window3d.m_joint[i].y, window3d.m_joint[i].z);
        //}
        //
        //for (int i = 0; i < window3d.m_joint2.size(); i++) {
        //    printf("camera 2, joint %d, %f, %f, %f\n ", window3d.m_joint2[i].num, window3d.m_joint2[i].x, window3d.m_joint2[i].y, window3d.m_joint2[i].z);
        //}
        //
        //for (int i = 0; i < window3d.m_joint3.size(); i++) {
        //    printf("camera 3, joint %d, %f, %f, %f\n ", window3d.m_joint3[i].num, window3d.m_joint3[i].x, window3d.m_joint3[i].y, window3d.m_joint3[i].z);
        //}
        //
        //for (int i = 0; i < window3d.m_joint4.size(); i++) {
        //    printf("camera 4, joint %d, %f, %f, %f\n ", window3d.m_joint4[i].num, window3d.m_joint4[i].x, window3d.m_joint4[i].y, window3d.m_joint4[i].z);
        //}
        //
        //for (int i = 0; i < window3d.m_joint5.size(); i++) {
        //    printf("camera 5, joint %d, %f, %f, %f\n ", window3d.m_joint5[i].num, window3d.m_joint5[i].x, window3d.m_joint5[i].y, window3d.m_joint5[i].z);
        //}
        //
        //for (int i = 0; i < window3d.m_joint6.size(); i++) {
        //    printf("camera 6, joint %d, %f, %f, %f\n ", window3d.m_joint6[i].num, window3d.m_joint6[i].x, window3d.m_joint6[i].y, window3d.m_joint6[i].z);
        //}
        //
        //for (int i = 0; i < window3d.m_joint7.size(); i++) {
        //    printf("camera 7, joint %d, %f, %f, %f\n ", window3d.m_joint7[i].num, window3d.m_joint7[i].x, window3d.m_joint7[i].y, window3d.m_joint7[i].z);
        //}
        //
        //for (int i = 0; i < window3d.m_joint8.size(); i++) {
        //    printf("camera 8, joint %d, %f, %f, %f\n ", window3d.m_joint8[i].num, window3d.m_joint8[i].x, window3d.m_joint8[i].y, window3d.m_joint8[i].z);
        //}

        //clock_t end = clock(); // 코드가 끝난 시간 저장
        //printf("buffer : %lf\n", (double)(end - start) / CLOCKS_PER_SEC);
      

        //joint memcpy
        if (update_is == 1) {
            vector<Window3dWrapper::jointcopy> jcopy1;
            copy(window3d.m_joint.begin(), window3d.m_joint.end(), back_inserter(jcopy1));
            jcopy1.assign(window3d.m_joint.begin(), window3d.m_joint.end()); // 전체 복사

            vector<Window3dWrapper::jointcopy> jcopy2;
            copy(window3d.m_joint2.begin(), window3d.m_joint2.end(), back_inserter(jcopy2));
            jcopy2.assign(window3d.m_joint2.begin(), window3d.m_joint2.end()); // 전체 복사

            vector<Window3dWrapper::jointcopy> jcopy3;
            copy(window3d.m_joint3.begin(), window3d.m_joint3.end(), back_inserter(jcopy3));
            jcopy3.assign(window3d.m_joint3.begin(), window3d.m_joint3.end()); // 전체 복사

            vector<Window3dWrapper::jointcopy> jcopy4;
            copy(window3d.m_joint4.begin(), window3d.m_joint4.end(), back_inserter(jcopy4));
            jcopy4.assign(window3d.m_joint4.begin(), window3d.m_joint4.end()); // 전체 복사

            vector<Window3dWrapper::jointcopy> jcopy5;
            copy(window3d.m_joint5.begin(), window3d.m_joint5.end(), back_inserter(jcopy5));
            jcopy5.assign(window3d.m_joint5.begin(), window3d.m_joint5.end()); // 전체 복사

            vector<Window3dWrapper::jointcopy> jcopy6;
            copy(window3d.m_joint6.begin(), window3d.m_joint6.end(), back_inserter(jcopy6));
            jcopy6.assign(window3d.m_joint6.begin(), window3d.m_joint6.end()); // 전체 복사

            vector<Window3dWrapper::jointcopy> jcopy7;
            copy(window3d.m_joint7.begin(), window3d.m_joint7.end(), back_inserter(jcopy7));
            jcopy7.assign(window3d.m_joint7.begin(), window3d.m_joint7.end()); // 전체 복사

            vector<Window3dWrapper::jointcopy> jcopy8;
            copy(window3d.m_joint8.begin(), window3d.m_joint8.end(), back_inserter(jcopy8));
            jcopy8.assign(window3d.m_joint8.begin(), window3d.m_joint8.end()); // 전체 복사

            joint_data.push(jcopy1);
            joint_data.push(jcopy2);
            joint_data.push(jcopy3);
            joint_data.push(jcopy4);
            joint_data.push(jcopy5);
            joint_data.push(jcopy6);
            joint_data.push(jcopy7);
            joint_data.push(jcopy8);

            //pointcloud memcpy
            vector<Visualization::PointCloudVertex> pcopy1;
            copy(window3d.m_pointClouds.begin(), window3d.m_pointClouds.end(), back_inserter(pcopy1));
            pcopy1.assign(window3d.m_pointClouds.begin(), window3d.m_pointClouds.end()); // 전체 복사

            vector<Visualization::PointCloudVertex> pcopy2;
            copy(window3d.m_pointClouds2.begin(), window3d.m_pointClouds2.end(), back_inserter(pcopy2));
            pcopy2.assign(window3d.m_pointClouds2.begin(), window3d.m_pointClouds2.end()); // 전체 복사

            vector<Visualization::PointCloudVertex> pcopy3;
            copy(window3d.m_pointClouds3.begin(), window3d.m_pointClouds3.end(), back_inserter(pcopy3));
            pcopy3.assign(window3d.m_pointClouds3.begin(), window3d.m_pointClouds3.end()); // 전체 복사

            vector<Visualization::PointCloudVertex> pcopy4;
            copy(window3d.m_pointClouds4.begin(), window3d.m_pointClouds4.end(), back_inserter(pcopy4));
            pcopy4.assign(window3d.m_pointClouds4.begin(), window3d.m_pointClouds4.end()); // 전체 복사

            vector<Visualization::PointCloudVertex> pcopy5;
            copy(window3d.m_pointClouds5.begin(), window3d.m_pointClouds5.end(), back_inserter(pcopy5));
            pcopy5.assign(window3d.m_pointClouds5.begin(), window3d.m_pointClouds5.end()); // 전체 복사

            vector<Visualization::PointCloudVertex> pcopy6;
            copy(window3d.m_pointClouds6.begin(), window3d.m_pointClouds6.end(), back_inserter(pcopy6));
            pcopy6.assign(window3d.m_pointClouds6.begin(), window3d.m_pointClouds6.end()); // 전체 복사

            vector<Visualization::PointCloudVertex> pcopy7;
            copy(window3d.m_pointClouds7.begin(), window3d.m_pointClouds7.end(), back_inserter(pcopy7));
            pcopy7.assign(window3d.m_pointClouds7.begin(), window3d.m_pointClouds7.end()); // 전체 복사

            vector<Visualization::PointCloudVertex> pcopy8;
            copy(window3d.m_pointClouds8.begin(), window3d.m_pointClouds8.end(), back_inserter(pcopy8));
            pcopy8.assign(window3d.m_pointClouds8.begin(), window3d.m_pointClouds8.end()); // 전체 복사

            point_cloud_data.push(pcopy1);
            point_cloud_data.push(pcopy2);
            point_cloud_data.push(pcopy3);
            point_cloud_data.push(pcopy4);
            point_cloud_data.push(pcopy5);
            point_cloud_data.push(pcopy6);
            point_cloud_data.push(pcopy7);
            point_cloud_data.push(pcopy8);
        }

        //clock_t start = clock();
        //start = clock();
		//window3d.SetLayout3d(s_layoutMode);
		
        //window3d.ConcateBuffer();
        window3d.ConcatePoint();
        std::stringstream ss_ply;

        if (update_is == 1) {
            string file_name_ply = to_string(iteration) + "_pointcloud.ply";
            if (window3d.m_pointClouds.size() > 0) {

                ss_ply << "ply" << std::endl;
                ss_ply << "format ascii 1.0" << std::endl;
                ss_ply << "element vertex"
                    << " " << window3d.m_pointClouds.size() << std::endl;

                ss_ply << "property float x" << std::endl;
                ss_ply << "property float y" << std::endl;
                ss_ply << "property float z" << std::endl;
                ss_ply << "property uchar red" << std::endl;
                ss_ply << "property uchar green" << std::endl;
                ss_ply << "property uchar blue" << std::endl;
                ss_ply << "end_header" << std::endl;

                for (int i = 0; i < window3d.m_pointClouds.size(); i++) {
                    ss_ply << window3d.m_pointClouds[i].Position[0] << " "
                        << window3d.m_pointClouds[i].Position[1] << " "
                        << window3d.m_pointClouds[i].Position[2] << " "
                        << (int)floor(window3d.m_pointClouds[i].Color[0] * 256) << " "
                        << (int)floor(window3d.m_pointClouds[i].Color[1] * 256) << " "
                        << (int)floor(window3d.m_pointClouds[i].Color[2] * 256) << " "
                        << std::endl;
                }

                std::ofstream ofs_text_ply(file_name_ply, std::ios::out | std::ios::app);
                ofs_text_ply.write(ss_ply.str().c_str(), (std::streamsize)ss_ply.str().length());
            }
        }
        update_is = 0;

        //printf("p size : %d\n", window3d.m_pointClouds.size());

        window3d.SetLayout3d(s_layoutMode);
        window3d.SetJointFrameVisualization(s_visualizeJointFrame);
        window3d.Render();
        
        //end = clock(); // 코드가 끝난 시간 저장
        //printf("rendering : %lf\n", (double)(end - start) / CLOCKS_PER_SEC);
                
	}
    save_thread.join();
    save_thread_joint.join();

    std::cout << "Finished body tracking processing!" << std::endl;

    window3d.Delete();
    
    k4abt_tracker_shutdown(tracker);
    k4abt_tracker_destroy(tracker);
    k4abt_tracker_shutdown(tracker2);
    k4abt_tracker_destroy(tracker2);
    k4abt_tracker_shutdown(tracker3);
    k4abt_tracker_destroy(tracker3);
    k4abt_tracker_shutdown(tracker4);
    k4abt_tracker_destroy(tracker4);
    k4abt_tracker_shutdown(tracker5);
    k4abt_tracker_destroy(tracker5);
    k4abt_tracker_shutdown(tracker6);
    k4abt_tracker_destroy(tracker6); 
    k4abt_tracker_shutdown(tracker7);
    k4abt_tracker_destroy(tracker7);
    k4abt_tracker_shutdown(tracker8);
    k4abt_tracker_destroy(tracker8);

    k4a_device_stop_cameras(device);
    k4a_device_close(device);
    k4a_device_stop_cameras(device2);
    k4a_device_close(device2);
    k4a_device_stop_cameras(device3);
    k4a_device_close(device3);
    k4a_device_stop_cameras(device4);
    k4a_device_close(device4);    
    k4a_device_stop_cameras(device5);
    k4a_device_close(device5);
    k4a_device_stop_cameras(device6);
    k4a_device_close(device6);
    k4a_device_stop_cameras(device7);
    k4a_device_close(device7);
    k4a_device_stop_cameras(device8);
    k4a_device_close(device8);
    

    return 0;
}
