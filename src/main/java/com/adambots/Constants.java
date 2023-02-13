/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.adambots;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */

public final class Constants {

    public static ShuffleboardTab debugTab;

    public static final class DriveConstants {
        public static final boolean kFrontLeftTurningEncoderReversed = true; // false
        public static final boolean kRearLeftTurningEncoderReversed = false;
        public static final boolean kFrontRightTurningEncoderReversed = true; // false
        public static final boolean kRearRightTurningEncoderReversed = false;

        public static final boolean kFrontLeftDriveEncoderReversed = false;
        public static final boolean kRearLeftDriveEncoderReversed = true;
        public static final boolean kFrontRightDriveEncoderReversed = false;
        public static final boolean kRearRightDriveEncoderReversed = true;

        // In Meters
        // Distance between centers of right and left wheels on robot
        public static final double kTrackWidth = 0.61; // 0.5
        // Distance between front and back wheels on robot
        public static final double kWheelBase = 0.61; // 0.7

        // Distance between front and back wheels on robot
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        public static final boolean kGyroReversed = true;

        // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
        // These characterization values MUST be determined either experimentally or
        // theoretically
        // for *your* robot's drive.
        // The SysId tool provides a convenient method for obtaining these values for
        // your robot.
        public static final double ksVolts = 1;
        public static final double kvVoltSecondsPerMeter = 0.8;
        public static final double kaVoltSecondsSquaredPerMeter = 0.15;

        public static final double kMaxSpeedMetersPerSecond = 3;
    }

    public static final class ModuleConstants {
        public static final double kMaxModuleAngularSpeedRadiansPerSecond = 16 * Math.PI;
        public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 16 * Math.PI;

        public static final int kEncoderCPR = 4096;
        public static final double kWheelDiameterMeters = 0.15;
        public static final double kDriveEncoderDistancePerPulse =
                // Assumes the encoders are directly mounted on the wheel shafts
                (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;
        public static final double kDriveEncoderScale = 0.04957;
 

        public static final double kTurningEncoderDistancePerPulse =
                // Assumes the encoders are on a 1:1 reduction with the module shaft.
                (2 * Math.PI) / (double) kEncoderCPR;

        public static double kPModuleTurningController = -1.1; //0.9
        public static double kIModuleTurningController = 0;
        public static double kDModuleTurningController = -0.01; //0.02
        public static final String kPTurningKey = "kPTurningKey";
        public static final String kDTurningKey = "kDTurningKey";

        public static final double kPModuleDriveController = 1;
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 2.3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 1.5;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        public static final double kPXController = 2.7;
        public static final double kPYController = 2.7;
        public static final double kPThetaController = 0.4;
    	public static final double kGyroTolerance = 0.5; //degrees tolerance for measurement

        // Constraint for the motion profiled robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static final class VisionConstants {
        public static final int kBackCamNumber = 1;
        public static final int kFrontCamNumber = 0;
        public static final int kCamExposure = 5;
        public static final int kFrameWidth = 320;
        public static final int kFrameHeight = 240;
        public static final int kHorizontalFOVDegrees = 60;
        public static final double kHorizontalDegreesPerPixel = (double) kHorizontalFOVDegrees / kFrameWidth;
        public static final int kImageHorizontalMidPoint = kFrameWidth / 2;
        public static final int kDriverStationFramesPerSec = 6;
        public static final int kProcessingFramesPerSec = 30; // DON'T CHANGE

        public static final double kCameraFieldOfView = 68.5;

    }

    

    public static final class GamepadConstants {
        
        // deadzone
        public static final double kDeadZone = 0.15;
    
        /**
         * Primary Driver Controller Port Number.
         */
        public static final int kPrimaryDriver = RobotMap.kPrimaryControllerPort;
        /**
         * Secondary Driver Controller Port Number.
         */
        public static final int kSecondaryDriver = RobotMap.kSecondaryControllerPort;
        /**
         * XBOX 360 South Face Button
         */
        public static final int kButtonA = 1;
        /**
         * XBOX 360 East Face Button
         */
        public static final int kButtonB = 2;
        /**
         * XBOX 360 West Face Button
         */
        public static final int kButtonX = 3;
        /**
         * XBOX 360 North Face Button
         */
        public static final int kButtonY = 4;
        /**
         * XBOX 360 Left Bumper (Top)
         */
        public static final int kButtonLB = 5;
        /**
         * XBOX 360 Right Bumper (Top)
         */
        public static final int kButtonRB = 6;
        /**
         * XBOX 360 Back Button
         */
        public static final int kButtonBack = 7;
        /**
         * XBOX 360 Start Button
         */
        public static final int kButtonStart = 8;
        /**
         * XBOX 360 Left Stick Click Button
         */
        public static final int kButtonLeftStick = 9;
        /**
         * XBOX 360 Right Stick Click Button
         */
        public static final int kButtonRightStick = 10;
    
        /**
         * XBOX 360 Left Horizontal Axis (Left=-1, Right=1)
         */
        public static final int kAxisLeftX = 0;
        /**
         * XBOX 360 Left Vertical Axis (Up=1, Down=-1)
         */
        public static final int kAxisLeftY = 1;
        /**
         * XBOX 360 Trigger Axis (LEFT)
         */
        public static final int kLeftAxisTriggers = 2;
        /**
         * XBOX 360 Trigger Axis (RIGHT)
         */
        public static final int kRightAxisTriggers = 3;
        /**
         * XBOX 360 Right Horizontal Axis (Left=-1, Right=1)
         */
        public static final int kAxisRightX = 4;
        /**
         * XBOX 360 Right Vertical Axis (Up=1, Down=-1)
         */
        public static final int kAxisRightY = 5;
    
        // the ID/port for the whole DPad
        // POV returns an angle in degrees 0-315 at 45 intervals
        public static final int kAxisDpadPov = 0;
    
        public static final int kDpadNAngle = 0;
        public static final int kDpadNEAngle = 45;
        public static final int kDpadEAngle = 90;
        public static final int kDpadSEAngle = 135;
        public static final int kDpadSAngle = 180;
        public static final int kDpadSWAngle = 225;
        public static final int kDpadWAngle = 270;
        public static final int kDpadNWAngle = 315;
    }

    public static final class GrabbyConstants {

        public static final int armEncoderCPR = 2048;
        public static final int mech2dAdjustment = +45;
        
        // misc.
        public static final double initiaLifterValue = 192;
        public static final double initialFirstExtenderValue = 0;
        public static final double initialSecondExtenderValue = 0;

        public static final double groundLifterValue = 80;
        public static final double groundFirstExtenderValue = 0;
        public static final double groundSecondExtenderValue = 0;

        // cube
        public static final double midCubeLifterValue = 128.6;
        public static final double midCubeFirstExtenderValue = 2;
        public static final double midCubeSecondExtenderValue = 0;

        public static final double highCubeLifterValue = 136.2;
        public static final double highCubeFirstExtenderValue = 3;
        public static final double highCubeSecondExtenderValue = 3;
        
        // cone
        public static final double midConeLifterValue = 146.2;
        public static final double midConeFirstExtenderValue = 2.5;
        public static final double midConeSecondExtenderValue = 0;

        public static final double highConeLifterValue = 146.2;
        public static final double highConeFirstExtenderValue = 3.5;
        public static final double highConeSecondExtenderValue = 3.5;

        public static class State{
            double armLiftTarget;
            double firstExtendTarget;
            double secondExtendTarget;
            public State(double armLiftTarget, double firstExtendTarget, double secondExtendTarget){
                this.armLiftTarget = armLiftTarget;
                this.firstExtendTarget = firstExtendTarget;
                this.secondExtendTarget = secondExtendTarget;
            }

            public double getArmLiftTarget(){
                return armLiftTarget;
            }

            public double getFirstExtendTarget(){
                return firstExtendTarget;
            }

            public double getSecondExtendTarget(){
                return secondExtendTarget;
            }
        }

        public static final State initState = new State(initiaLifterValue, initialFirstExtenderValue, initialSecondExtenderValue);
        public static final State groundState = new State(groundLifterValue, groundFirstExtenderValue, groundSecondExtenderValue);
        public static final State midCubeState = new State(midCubeLifterValue, midCubeFirstExtenderValue, midCubeSecondExtenderValue);
        public static final State midConeState = new State(midConeLifterValue, midConeFirstExtenderValue, midConeSecondExtenderValue);
        public static final State highCubeState = new State(highCubeLifterValue, highCubeFirstExtenderValue, highCubeSecondExtenderValue);
        public static final State highConeState = new State(highConeLifterValue, highConeFirstExtenderValue, highConeSecondExtenderValue);


        //general
        public static final double firstExtenderMaxExtend = 6;
        public static final double secondExtenderMaxExtend = 6;
        public static final double rotationPerInch = 1;

        // antiquated
        public static final double midZoneLifterValue = 0.5;
        public static final double midZoneExtenderValue = 0.5;

        public static final double highZoneLifterValue = 1;
        public static final double highZoneExtenderValue = 1;

        public static final double lifterSpeed = 0.25;
        public static final double extenderSpeed = 0.25;
    }


    
}
