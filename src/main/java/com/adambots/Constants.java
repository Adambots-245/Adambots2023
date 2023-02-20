/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.adambots;

import org.opencv.core.Scalar;

import java.lang.reflect.Field;
import java.util.Map;

import com.adambots.utils.ModuleMap;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

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
        public static Field2d field = new Field2d();

        public static final boolean kFrontLeftTurningEncoderReversed = true; // false
        public static final boolean kRearLeftTurningEncoderReversed = false;
        public static final boolean kFrontRightTurningEncoderReversed = true; // false
        public static final boolean kRearRightTurningEncoderReversed = false;

        public static final boolean kFrontLeftDriveEncoderReversed = false;
        public static final boolean kRearLeftDriveEncoderReversed = false;
        public static final boolean kFrontRightDriveEncoderReversed = false;
        public static final boolean kRearRightDriveEncoderReversed = false;

        // In Meters
        // Distance between centers of right and left wheels on robot
        public static final double kTrackWidth = 0.61; // 0.5
        // Distance between front and back wheels on robot
        public static final double kWheelBase = 0.61; // 0.7

        public enum ModulePosition {
            FRONT_LEFT,
            FRONT_RIGHT,
            REAR_LEFT,
            REAR_RIGHT
          }
      
          public static final Map<ModulePosition, Translation2d> kModuleTranslations = Map.of(
              ModulePosition.FRONT_LEFT, new Translation2d(kWheelBase / 2, kTrackWidth / 2),
              ModulePosition.FRONT_RIGHT, new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
              ModulePosition.REAR_LEFT, new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
              ModulePosition.REAR_RIGHT, new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));
      
        //   public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        //       ModuleMap.orderedValues(DriveConstants.kModuleTranslations, new Translation2d[0]));
      

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

        public static final double kMaxSpeedMetersPerSecond = 3; //Only used in Drive and SetModuleStates

        public static final double kSpeedThreshold = 0.001; //Minimum Speed for Swerve Modules
        public static int kOffBalanceAngleThresholdDegrees = 3;
        public static int kOonBalanceAngleThresholdDegrees = 3;
    }

    public static final class ModuleConstants {
        public static final double kMaxModuleAngularSpeedRadiansPerSecond = 16 * Math.PI; //Limits for wheel turning profiled PID
        public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 16 * Math.PI;

        public static final int kEncoderCPR = 4096;
        public static final double kWheelDiameterMeters = 0.15;
        public static final double kDriveEncoderDistancePerPulse =
                // Assumes the encoders are directly mounted on the wheel shafts
                (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;
        public static final double kDriveEncoderScale = 0.0470915;
 

        public static final double kTurningEncoderDistancePerPulse =
                // Assumes the encoders are on a 1:1 reduction with the module shaft.
                (2 * Math.PI) / (double) kEncoderCPR;

        public static double kPModuleTurningController = -1.1; //PID Values for angular wheel rotation
        public static double kIModuleTurningController = 0;
        public static double kDModuleTurningController = -0.01; //0.02

        public static final double kPModuleDriveController = 1; //P 0 0 for drive controller of swerve module
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 2.3; //Max speed of auton
        public static final double kMaxAccelerationMetersPerSecondSquared = 1.5; //Max acceleration of auton
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI; //Max rotational speed of auton
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI; //Max rotational acceleration of auton

        public static final double kPXController = 2.7; // P 0 0 values for auton X, Y, and Theta controllers
        public static final double kPYController = 2.7;
        public static final double kDXController = 0.11; // P 0 0 values for auton X, Y, and Theta controllers
        public static final double kDYController = 0.11;
        public static final double kPThetaController = 0.55;
        public static final double kDThetaController = 0.05;


        // Constraint for the motion profiled robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

        public static final Map<Integer, Pose3d> aprilTags =
        Map.of(
          1,
          new Pose3d(
              Units.inchesToMeters(610.77),
              Units.inchesToMeters(42.19),
              Units.inchesToMeters(18.22),
              new Rotation3d(0.0, 0.0, Math.PI)),
          2,
          new Pose3d(
              Units.inchesToMeters(610.77),
              Units.inchesToMeters(108.19),
              Units.inchesToMeters(18.22),
              new Rotation3d(0.0, 0.0, Math.PI)),
          3,
          new Pose3d(
              Units.inchesToMeters(610.77),
              Units.inchesToMeters(174.19), // FIRST's diagram has a typo (it says 147.19)
              Units.inchesToMeters(18.22),
              new Rotation3d(0.0, 0.0, Math.PI)),
          4,
          new Pose3d(
              Units.inchesToMeters(636.96),
              Units.inchesToMeters(265.74),
              Units.inchesToMeters(27.38),
              new Rotation3d(0.0, 0.0, Math.PI)),
          5,
          new Pose3d(
              Units.inchesToMeters(14.25),
              Units.inchesToMeters(265.74),
              Units.inchesToMeters(27.38),
              new Rotation3d()),
          6,
          new Pose3d(
              Units.inchesToMeters(40.45),
              Units.inchesToMeters(174.19), // FIRST's diagram has a typo (it says 147.19)
              Units.inchesToMeters(18.22),
              new Rotation3d()),
          7,
          new Pose3d(
              Units.inchesToMeters(40.45),
              Units.inchesToMeters(108.19),
              Units.inchesToMeters(18.22),
              new Rotation3d()),
          8,
          new Pose3d(
              Units.inchesToMeters(40.45),
              Units.inchesToMeters(42.19),
              Units.inchesToMeters(18.22),
              new Rotation3d()));
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
        public static final Scalar RED = new Scalar(0, 0, 255);
        public static final Scalar GREEN = new Scalar(0, 255, 0);
        public static final Scalar BLUE = new Scalar(255, 0, 0);
        public static final int IMG_WIDTH = 320;
        public static final int HOR_FOV_DEGREES = 60;
        public static final int IMG_HOR_MID = IMG_WIDTH / 2;
        public static final double HOR_DEGREES_PER_PIXEL = (double) HOR_FOV_DEGREES / IMG_WIDTH;

        public static final double kCameraFieldOfView = 68.5;

    }

    public static final class PreferencesConstants {
        public static final String kPModuleTurningKey = "kPModuleTurningKey";
        public static final String kDModuleTurningKey = "kDModuleTurningKey";
        public static final String kMaxSpeedMetersPerSecondKey = "kMaxSpeedMetersPerSecondKey";
        public static final String kMaxAccelerationMetersPerSecondSquaredKey = "kMaxAccelerationMetersPerSecondSquaredKey";
        public static final String kMaxAngularSpeedRadiansPerSecondKey = "kMaxAngularSpeedRadiansPerSecondKey";
        public static final String kMaxAngularSpeedRadiansPerSecondSquaredKey = "kMaxAngularSpeedRadiansPerSecondSquaredKey";
        public static final String kPXControllerKey = "kPXControllerKey";
        public static final String kPYControllerKey = "kPYControllerKey";
        public static final String kPThetaController = "kPThetaController"; 
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
        public static final double initiaLifterValue = 212;
        public static final double initialFirstExtenderValue = 0;
        public static final double initialSecondExtenderValue = 0;

        public static final double groundLifterValue = 94;
        public static final double groundFirstExtenderValue = 0;
        public static final double groundSecondExtenderValue = 0;

        // cube
        public static final double midCubeLifterValue = 135.4;
        public static final double midCubeFirstExtenderValue = 38.56 * armEncoderCPR;
        public static final double midCubeSecondExtenderValue = 0;

        public static final double highCubeLifterValue = 143;
        public static final double highCubeFirstExtenderValue = 60.25 * armEncoderCPR;
        public static final double highCubeSecondExtenderValue = 3;
        
        // cone
        public static final double midConeLifterValue = 155.9;
        public static final double midConeFirstExtenderValue = 60.25 * armEncoderCPR;
        public static final double midConeSecondExtenderValue = 0;

        public static final double highConeLifterValue = 155.9;
        public static final double highConeFirstExtenderValue = 60.25 * armEncoderCPR;
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


        //PID values
        public static final double lifterP = 0.02;
        public static final double lifterI = 0.02;
        public static final double lifterD = 0.02;

        public static final double firstExtenderP = 0.02;
        public static final double firstExtenderI = 0.02;
        public static final double firstExtenderD = 0.02;

        public static final double secondExtenderP = 0.02;
        public static final double secondExtenderI = 0.02;
        public static final double secondExtenderD = 0.02;

        //general
        public static final double firstExtenderMaxExtend = 60.25 * armEncoderCPR;
        public static final double secondExtenderMaxExtend = 59.5 * armEncoderCPR;
        public static final double rotationPerInch = 1;

        // antiquated
        public static final double midZoneLifterValue = 0.5;
        public static final double midZoneExtenderValue = 0.5;

        public static final double highZoneLifterValue = 1;
        public static final double highZoneExtenderValue = 1;

        public static final double lifterSpeed = 0.15;
        public static final double extenderSpeed = 0.25;
        public static final double armStopSpeed = 0.04;
    }


    
}
