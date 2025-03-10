#lifted from https://github.com/LimelightVision/limelightlib-python/tree/main

# look into making a subsystem out of this. Or just use ll imported.

# results specification: https://docs.limelightvision.io/docs/docs-limelight/apis/json-results-specification
# basic use: https://docs.limelightvision.io/docs/docs-limelight/apis/complete-networktables-api

# command based... floating subsystem.



import limelight
import limelightresults

from ntcore import NetworkTable
from robotUtils.limelight import LimelightHelpers
# import limelightHelpers (only in Java/C++ so far; could write it up)
import json
import time
from commands2 import Subsystem
from wpimath.geometry import Pose2d, Transform2d, Translation2d, Rotation2d
from wpimath.units import degreesToRadians
# from wpilib import SmartDashboard   
import ntcore as nt
import numpy as np
from phoenix6.utils import get_current_time_seconds
from phoenix6 import SignalLogger
from wpilib import SmartDashboard
from constants import LimelightConstants


class LimelightSubsystem(Subsystem):
    def __init__(self):
        super().__init__()
        # self.NTinstance = NetworkTable.getInstance()
        # self.ll3Table = self.NTinstance.getTable("limelight")
        # self.ll4Table = self.NTinstance.getTable("limelight-four")

        self.discovered_limelights = limelight.discover_limelights()#debug=True)
        print("discovered limelights:", self.discovered_limelights)
        with open(LimelightConstants.field_map_address,'r') as mapjson:
            self.field_map =   mapjson.read()

        if self.discovered_limelights:
            self.qty_limelights = len(self.discovered_limelights)
            print("'\n-------------\n------------------\n'qty limelights:", self.qty_limelights,'\n-------------\n------------------\n')
            
            limelight_address_1 = self.discovered_limelights[0] #TODO make us able to use multiple limelights; maybe a second subsystem
            self.ll = limelight.Limelight(limelight_address_1)
            First_limelight_name = self.ll.get_name()
            SmartDashboard.putString('First Limelight Name', First_limelight_name)
            First_LL_FIELD_UPLOAD_STATUS = self.ll.upload_fieldmap(self.field_map, index=None)
            SmartDashboard.putString(f'{First_limelight_name} Limelight map upload attempt', First_LL_FIELD_UPLOAD_STATUS.reason)

            if self.qty_limelights > 1:
                limelight_address_2 = self.discovered_limelights[1]
                self.ll2 = limelight.Limelight(limelight_address_2)
                Second_limelight_name = self.ll2.get_name()
                SmartDashboard.putString('Second Limelight Name', Second_limelight_name)
                Second_LL_FIELD_UPLOAD_STATUS = self.ll2.upload_fieldmap(self.field_map, index=None)
                SmartDashboard.putString(f'{Second_limelight_name} Limelight map upload attempt', Second_LL_FIELD_UPLOAD_STATUS.reason)

            # LL4 IMU Modes:
            # 0 : use external submitted via set_robot_orientation; ignore interal completely
            # 1 : use external submitted via set_robot_orientation; use internal fused yaw
            # 2 : use internal yaw; ignore external completely
            # IMU Mode 3 - IMU_ASSIST_MT1 - The internal IMU will utilize filtered MT1 yaw estimates for continuous heading correction
            # IMU Mode 4 - IMU_ASSIST_EXTERNALIMU - The internal IMU will utilize the external IMU for continuous heading correction
            
            LimelightHelpers.set_imu_mode('limelight', mode=1) #TODO: this doesn't make sense for a LL3
            LimelightHelpers.set_camerapose_robotspace('limelight', forward=LimelightConstants.kLL3forward,
                                                       side=LimelightConstants.kLL3side,
                                                       up=LimelightConstants.kLL3up,
                                                       roll=LimelightConstants.kLL3roll,
                                                       pitch=LimelightConstants.kLL3pitch,
                                                       yaw=-LimelightConstants.kLL3yaw)
                                                                   
            if self.qty_limelights > 1:
                
                LimelightHelpers.set_imu_mode('limelight-four', mode=1)
                LimelightHelpers.set_limelight_NTDouble("limelight-four", "imuassistalpha_set", 0.01) # default is 0.001
                SmartDashboard.putNumber('LL4 IMU mode', LimelightHelpers.get_limelight_NTDouble("limelight-four", "imumode_set"))
                LimelightHelpers.set_camerapose_robotspace('limelight-four', forward=LimelightConstants.kLL4forward,
                                                       side=LimelightConstants.kLL4side,
                                                       up=LimelightConstants.kLL4up,
                                                       roll=LimelightConstants.kLL4roll,
                                                       pitch=LimelightConstants.kLL4pitch,
                                                       yaw=-LimelightConstants.kLL4yaw)
                LimelightHelpers.set_limelight_NTDouble("limelight-four", 'throttle_set',100)
            results = self.ll.get_results()
            status = self.ll.get_status()
            print("-----")
            print("targeting results:", results)
            print("-----")
            print("status:", status)
            print("-----")
            print("temp:", self.ll.get_temp())
            print("-----")
            print("name:", self.ll.get_name())
            print("-----")
            print("fps:", self.ll.get_fps())
            print("-----")
            print("hwreport:", self.ll.hw_report())

            
            strip_MT1_ll = self.stripped_down_pose(LimelightHelpers.get_botpose_estimate_wpiblue('limelight'))
            strip_MT2_ll = self.stripped_down_pose(LimelightHelpers.get_botpose_estimate_wpiblue_megatag2('limelight'))
            strip_MT1_ll4 = self.stripped_down_pose(LimelightHelpers.get_botpose_estimate_wpiblue('limelight-four'))
            strip_MT2_ll4 = self.stripped_down_pose(LimelightHelpers.get_botpose_estimate_wpiblue_megatag2('limelight-four'))
            SignalLogger.write_double_array("limelight MT1 Pose", strip_MT1_ll)
            SignalLogger.write_double_array("limelight MT2 Pose", strip_MT2_ll)
            SignalLogger.write_double_array("limelight-four MT1 Pose", strip_MT1_ll4)
            SignalLogger.write_double_array("limelight-four MT2 Pose", strip_MT2_ll4)
            
            self.ll.enable_websocket()
            if self.qty_limelights>1:
                self.ll2.enable_websocket()
            # print(self.ll.get_pipeline_atindex(0))
            

    def stripped_down_pose(self, botpose):
        '''return the pose (only) as a float array'''
        #return PoseEstimate(pose, adjusted_timestamp, latency, tag_count, tag_span, tag_dist, tag_area, raw_fiducials, is_megatag_2)
        stripped_pose= [botpose.pose.translation().x, botpose.pose.translation().y, botpose.pose.rotation().degrees()]
        return stripped_pose

    #update python (on limelight) inputs
    def update_python_inputs(self, inputs):
        return self.ll.update_python_inputs(inputs)

    def get_results(self):
        return self.ll.get_results()
    
    # get latest results
    def get_latest_results(self):
        return self.ll.get_latest_results()

    def get_status(self):
        return self.ll.get_status()

    def get_temp(self):
        return self.ll.get_temp()

    def get_name(self):
        return self.ll.get_name()

    def get_fps(self):
        return self.ll.get_fps()

    def hw_report(self):
        return self.ll.hw_report()

    def get_pipeline_atindex(self, index):
        return self.ll.get_pipeline_atindex(index)

    def update_pipeline(self, pipeline_update, flush):
        return self.ll.update_pipeline(pipeline_update, flush)

    def pipeline_switch(self, index):
        return self.ll.pipeline_switch(index)

    def update_python_inputs(self, inputs):
        return self.ll.update_python_inputs(inputs)

    def get_latest_results(self):
        return self.ll.get_latest_results()

    def disable_websocket(self):
        return self.ll.disable_websocket()

    def enable_websocket(self):
        return self.ll.enable_websocket()
    
    def set_imu_mode_align(self):
        return self.ll.set_imu_mode(1)
    def set_imu_mode_independent(self):
        return self.ll.set_imu_mode(2)
    
    
    #section specialty utilities
    '''def trust_target(self, robot_pose: Pose2d, residual_threshold=1000000000, override_and_trust=True): #TODO: fix threshold
        #calculate distance to the detected target
        #will the results hav only one target's results? assume one for now
        # Done: check unit consistency: botpose is in meters, robot_pose is in meters
        trust_vision_data = True
        if self.discovered_limelights:
            latest_parsed_result = limelightresults.parse_results(self.ll.get_latest_results())
            self.ll 
            detector_confidences = [detector.confidence for detector in latest_parsed_result.detectorResults]
            # print("detector confidences:", detector_confidences) Commented out by Sebastian because it was annoying
            if latest_parsed_result:
                validity = latest_parsed_result.validity #what units are validity in ?
                trust_vision_data *= (validity==1) 
                vision_bot_pose_to_use = latest_parsed_result.botpose_wpiblue #botpose

                delta_x  = vision_bot_pose_to_use[0] - robot_pose.x
                delta_y = vision_bot_pose_to_use[1] - robot_pose.y
                residual = np.sqrt(delta_x**2 + delta_y**2)
                latency = latest_parsed_result.targeting_latency
                time_of_measurement = get_current_time_seconds() - .001 * latency #only acounting for json unpacking 
 
                trust_vision_data *= (residual < residual_threshold) 
                ''' '''General results do not have stddev -- need to use MegaTag2
                if latest_parsed_result.stddevs:
                    detect_stdDev = latest_parsed_result.stddevs
                    detect_stdDev_x = detect_stdDev[0]
                    detect_stdDev_y = detect_stdDev[1]
                    max_stdDev = max(detect_stdDev_x, detect_stdDev_y)
                    trust_vision_data *= (residual < 2 * max_stdDev) 
                    #if the residual is within 2 standard deviations, 
                    # and the residual is less than 1 meter, 
                    ''' '''
                # trust the target
                detect_stdDev_x = None
                detect_stdDev_y = None
                trust_telemetry = [validity, residual, detect_stdDev_x, detect_stdDev_y]
                # print(f"trust_telemetry:{trust_vision_data} -> {trust_telemetry}     IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII")
                
                if override_and_trust: trust_vision_data = True
                viz_pose = Pose2d(Translation2d(vision_bot_pose_to_use[0], vision_bot_pose_to_use[1]),
                                    Rotation2d(degreesToRadians(vision_bot_pose_to_use[5])))
        else:
            trust_vision_data = False
            viz_pose = None
            latest_parsed_result= None
            time_of_measurement = get_current_time_seconds()

        return (trust_vision_data , viz_pose, latest_parsed_result, time_of_measurement)'''


    #endsection specialty utilities

    def periodic(self):
        LL3_botPoseArrayBlue = LimelightHelpers.get_botpose_2d_wpiblue('limelight')
        LL3_botPoseArrayBlue =LimelightHelpers.pose_2d_to_array(LL3_botPoseArrayBlue)
        LL3_botPoseArrayBlueMT2 = LimelightHelpers.get_botpose_estimate_wpiblue_megatag2('limelight')
        LL3_botPoseArrayBlueMT2 = LimelightHelpers.pose_2d_to_array(LL3_botPoseArrayBlueMT2.pose)
        LL4_botPoseArrayBlue = LimelightHelpers.get_botpose_2d_wpiblue('limelight-four')
        LL4_botPoseArrayBlue = LimelightHelpers.pose_2d_to_array(LL4_botPoseArrayBlue)
        LL4_botPoseArrayBlueMT2 = LimelightHelpers.get_botpose_estimate_wpiblue_megatag2('limelight-four')
        LL4_botPoseArrayBlueMT2 = LimelightHelpers.pose_2d_to_array(LL4_botPoseArrayBlueMT2.pose)
        SmartDashboard.putNumberArray("LL3 Pose Blue", 
                                      LL3_botPoseArrayBlue)
        SmartDashboard.putNumberArray("LL3 Pose Blue MT2", 
                                      LL3_botPoseArrayBlueMT2)
        SmartDashboard.putNumberArray("LL4 Pose Blue", 
                                      LL4_botPoseArrayBlue)
        SmartDashboard.putNumberArray("LL4 Pose Blue MT2", 
                                      LL4_botPoseArrayBlueMT2)
        
        
        

    # This would not execute.. only commands have these methods
    # def autonoumousPeriodic(self):
    #     # LimelightHelpers.set_imu_mode('limelight', mode=2)
    #     LimelightHelpers.set_imu_mode('limelight-four', mode=0)
    #     SmartDashboard.putNumber('LL4 IMU mode', LimelightHelpers.get_limelight_NTDouble("limelight-four",  "imumode_set"))
    #     pass
    
    # def teleopPeriodic(self):
    #     # LimelightHelpers.set_imu_mode('limelight', mode=2)
    #     LimelightHelpers.set_imu_mode('limelight-four', mode=0)
    #     pass

    def end(self):
        if self.discovered_limelights: 
            self.ll.disable_websocket()
            if self.qty_limelights>1:
                self.ll2.disable_websocket()
    