## 1. issac data collector
    (1) sensor data usage, data collected from issac sim

        imu -> can_bus

        lidar -> point

        odom -> ego_pose

        camera -> 6xpng

        boundingbox3d -> annotation

    (2) synchronization
        issac use simulation time
        issac datas in the same frame are synchronized,pay attention to the frame lost situation
        we have a large effort on issac sim, and we use ros2 to get it
        sample_annotation-对齐-sample_data，sample_data-对齐-ego_pose。
        - nuscenes sample token: six lidar pcd and six camera png(当顶部 LIDAR 扫描相机 FOV 的中心时，会触发相机的曝光)
            (7x or 6x frame, one keyframe) radar_front, radar_front_left, radar_front_right, radar_back_left, radar_back_right,
            (10x frame, one keyframe) lidar
            (6x frame, one keyframe) CAN_FRONT,CAM_FRONT_LEFT,CAM_FRONT_RIGHT,CAM_BACK,CAM_BACK_LEFT,CAM_BACK_RIGHT

        - our method: 
            for img and bbox3d, we use ros2 message_filters to sync it greatly,each cam sample_data has an bbox3d annotation
            for lidar and img, we use the clock to sync them
            ***************ATTENTION:recod sample, scene, log only in CAM_FRONT callback********************

            sample is aligned strictly, sweeps not , and probably impossible to align
        

    (3) workflow

        simulation time: 1s/60hz (cam, lidar, odom)
            - set the step parpameter in issac to 1s/12hz, like nuscenes' capture frequency  # TODO

        nav_msgs/msg/odometry.hpp -> ego_pose
        sensor_msgs/msg/image.hpp -> sample_data
        vision_msgs/msg/detection3_d_array.hpp -> bbox3d

        collection sequence:
            - ego_pose and sample_data and sample_annotation together
            - set target_frame_count as the max frame which we can capture
            - Set sample_sensors_ as the device that should ensure the captured data from the device meets the target frame count requirement

        - capture or post processing json
            ·  'scene'：获取场景的元数据。获取地图，可以跳过                  (OK)
            ·  'log'：获取地图，可以跳过                                   (OK)
            ·  'calibrated_sensor'：获取传感器的校准信息。                  (OK)    
            ·  'ego_pose'：获取 Ego 车辆的位置和姿态信息。                   (OK)
            ·  'sample'：获取某个样本的元数据。                             (OK)
            ·  'sample_annotation'：获取样本的标注信息。                    (OK)
            ·  'sample_data'：获取样本数据，如 LIDAR、相机等传感器的数据。     (OK)
            ·  'instance'：场景中连续记录同一物体出现的时间，得到其帧之间的位置差，计算速度  (TODO: waitting for the category)
            ·  'can_bus'：获取样本的标注信息。                              (waitting_list)
            ·  'map':map which is semantic segmentation, use for supervise the map decoder. (OK)
        - definition json:
            ·   'attribute':            (TODO)
            ·   'calibrated_sensor':    (OK)
            ·   'category':             (TODO)
            ·   'sensor':               (OK)
            ·   'visibility':           (OK)


        capture data, write into json -> sample and write new json

        token formulas:  // this is for a unique token in simulation env which is predictable only in this run
            sample_data_token = harsh(device_frame_count, "samples_data_" + device_name, start_system_time_token_)  // frame help to record next token
            
            ego_pose_token = sample_data_token // every sample_data_token has its newest ego_pose
                
            sample_tokne = harsh(sample_frame_count, "sample", start_system_time_token_) //每个sample_token包含0.5秒内的所有sample_data，时间戳最适合的sample_data被标为key_frame且用于标注

            scene_token = "scene_" + map_name_ + "_" + current_system_time

            scene_name = scene_mode + cur_scene_count + start_system_time

            log_token = map_name_ + start_system_time

            // use instance_token map to record whether the same instance appeared in prev 
            sample_annotation_token = generate_unique_token(devices_frame_count_, "3dbox_" + device_name + instance_token, start_system_time_token_) // frame help to record next token, and classify obj detected in same frame

            Visibility_token 根据置信度生成(result:1,2,3,4)

            Instance_token = generate_unique_token(0, class_id, start_system_time_token_)
                - class_id = class + ‘_’ + id, such as people_jack

            prev and next token generation: 
            ****************nly sample_token generate by the prediction of rule, others generate by slider dynamically**************
                    scheme1(wasted): use map to generate bidirectional list like file content struture in real time   
                                - j1<->j2<->j3<->...<->jn+1
                                - future write back need to search json, with the json grow larger, this won't work well
                    scheme2(adopted) slider: use sliding window to generate(window size 2).
                                - (j1,j2)->(j2,j3)....->(jn,jn+1) = (no prev)j1<->j2<->j3<->......jn, post processing jn+1(no next)
                                - this algorithm will cost much effort
                    scheme3(wasted) rule prediction : use a formula which is predictable to generate token
                                - sample_data_token = harsh(frame, "samples_data_" + device_name, start_system_time_token_)
                                - the predictable variable is frame, others are fixed, and this is easy to code

    (4)save
        The data generated by this node will be saved in root_path/NuscenesData
        sample_annotations' last frame are cached
        instance will be generate when this scene done

    (5)error:
        **** error depend on whether data will be discarded because of the queue size is full ****
        misalignment time is frame level, default time is 1/60 each frame

        after test, maybe there is some frame lost, but i think it won't hurts a lot, data are aligned well

    (6)rule:
        1. when your use it to capture an epoch of data, you should backup your previous data.
        2. one epoch should be done by code ,not by yourslf, or you will get a wrong data in synchronizing.
        *3. the frame we sample, control the end of process

    (7)test: test about the capture and sync
        - capture or post processing json
            ·  'scene'：获取场景的元数据。                                      (OK)
                ·  'log':                                                    (OK)
                ·  'calibrated_sensor'：获取传感器的校准信息。                  (OK)    
                ·  'ego_pose'：获取 Ego 车辆的位置和姿态信息。                   (OK)
                ·  'sample'：获取某个样本的元数据。                             (OK)
                ·  'sample_annotation'：获取样本的标注信息。                    (OK)
                ·  'sample_data'：获取样本数据，如 LIDAR、相机等传感器的数据。     (OK)
                ·  'instance'：场景中连续记录同一物体出现的时间，得到其帧之间的位置差，计算速度  (TODO)
                ·  'can_bus' and can_bus package：获取样本的标注信息。                              (waitting_list)
                ·  'map' map and map package:map which is semantic segmentation, use for supervise the map decoder. (TODO)
        - definition json:
            ·   'attribute':            (TODO)
            ·   'calibrated_sensor':    (OK)
            ·   'category':             (TODO)
            ·   'sensor':               (OK)
            ·   'visibility':           (OK)

        - sync: after testing, the sync issac sim frame is right, if not, be careful to the pubisher you defined in issac sim
            TODO: cam sync has some problem, some cam got 285 frame, and other got 286, in 300 frame.

## 2. real world data collector

    1. sync:
        - use the same prefix in a frame, which contain 6 cam jpgs and 1 lidar bin
        #todo: unlike issac sim, each frame is not sync, so we should use timestamp or other measure to attempt synchronization
        - compare to issac sim, 使用 ROS2 ApproximateTimeSynchronizer 进行msg同步, make it have a synchronized frame like issac sim


## 3. start up

    -- set up Real-Time Publish-Subscribe, Fast RTPS, used for RT communication
    sudo apt-get install ros-humble-rmw-cyclonedds-cpp
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp