# 3D_Pose_Estimation_using-_Joint_based_Calibration_in_Distributed_RGBD_Camera_System

Github : https://github.com/BSParkICLAB/3D_Pose_Estimation_using-_Joint_based_Calibration_in_Distributed_RGBD_Camera_System 

- Requirements
  - Dependency
    - glfw.3.3.0
    - opencv4.10
    - cudnn64.7
    - dnn_model_2_0.onnx
 
  - Install
    - Azure Kinect DK Code Samples Repository
    - https://github.com/microsoft/Azure-Kinect-Samples/tree/master/body-tracking-samples
    - Azure Kinect Body Tracking SDK
    - https://learn.microsoft.com/en-us/azure/kinect-dk/body-sdk-download

  - Update code 
      - /code/simple_3d_viewer -> Optimizer Code
      - /code/sample_helper_libs/window_controller_3d -> Rendering Code
      - Please also note that to use the Body Tracking SDK with Unreal, make sure you have added <SDK Installation          Path\tools to the environment variable PATH and copied dnn_model_2_0.onnx and cudnn64_7.dll Release Folder
        <br/><br/>
- Dataset Google Drive : https://drive.google.com/drive/folders/18NpBvNmxNJI5xuTJ8G_0Ay_cT4fQUZ4k?usp=sharing

  - Dataset structure
      - Dataset_A_JH,HB (each 50 images x 8camera)
          - Joint
          - Pointcloud
          - RGB
          - Depth
      - Dataset_B_JH,HB (each 1000 images x 8camera)
          - RGB
          - Depth
<br/><br/>
      

![introduction-01](https://github.com/BSParkICLAB/3D_Pose_Estimation_using-_Joint_based_Calibration_in_Distributed_RGBD_Camera_System/assets/146613437/2907afe2-b6ea-4dae-b1e8-890fb360d772)
![introduction-04](https://github.com/BSParkICLAB/3D_Pose_Estimation_using-_Joint_based_Calibration_in_Distributed_RGBD_Camera_System/assets/146613437/1026ece1-82c4-48a0-960d-adf68e2b7f27)
![introduction-05](https://github.com/BSParkICLAB/3D_Pose_Estimation_using-_Joint_based_Calibration_in_Distributed_RGBD_Camera_System/assets/146613437/bde9d641-7c4a-4b63-9ea0-d9c1bbf8910d)
![introduction-06](https://github.com/BSParkICLAB/3D_Pose_Estimation_using-_Joint_based_Calibration_in_Distributed_RGBD_Camera_System/assets/146613437/82f09329-1363-4752-8ffc-8de1f7ada7b6)
![introduction-07](https://github.com/BSParkICLAB/3D_Pose_Estimation_using-_Joint_based_Calibration_in_Distributed_RGBD_Camera_System/assets/146613437/44fb49f0-b367-4f1e-8013-f1069e6cf753)
![introduction-08](https://github.com/BSParkICLAB/3D_Pose_Estimation_using-_Joint_based_Calibration_in_Distributed_RGBD_Camera_System/assets/146613437/57e93615-1d99-47e3-a603-b894eda87d3f)
![introduction-09](https://github.com/BSParkICLAB/3D_Pose_Estimation_using-_Joint_based_Calibration_in_Distributed_RGBD_Camera_System/assets/146613437/47df0bbc-3555-4c20-a584-77bede4f62e7)
![introduction-10](https://github.com/BSParkICLAB/3D_Pose_Estimation_using-_Joint_based_Calibration_in_Distributed_RGBD_Camera_System/assets/146613437/6724165c-78ff-47da-ad81-e661a513a129)
![introduction-11](https://github.com/BSParkICLAB/3D_Pose_Estimation_using-_Joint_based_Calibration_in_Distributed_RGBD_Camera_System/assets/146613437/b52aa89a-fc4e-40eb-a43a-3798f3076cb8)
![introduction-12](https://github.com/BSParkICLAB/3D_Pose_Estimation_using-_Joint_based_Calibration_in_Distributed_RGBD_Camera_System/assets/146613437/32d6df75-17d9-403a-a63e-ddb46559d847)
