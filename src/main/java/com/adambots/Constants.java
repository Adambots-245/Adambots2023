/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.adambots;

import java.util.Map;

import org.opencv.core.Scalar;

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
    public static final String kDefaultShuffleboardTab = "debug";
    public static ShuffleboardTab debugTab;

    public static final class DriveConstants {
        public static Field2d field = new Field2d();

        public static final boolean kFrontLeftTurningEncoderReversed = true;
        public static final boolean kRearLeftTurningEncoderReversed = false;
        public static final boolean kFrontRightTurningEncoderReversed = true;
        public static final boolean kRearRightTurningEncoderReversed = false;

        public static final boolean kFrontLeftDriveEncoderReversed = false;
        public static final boolean kRearLeftDriveEncoderReversed = false;
        public static final boolean kFrontRightDriveEncoderReversed = false;
        public static final boolean kRearRightDriveEncoderReversed = false;

        // In Meters
        // Distance between centers of right and left wheels on robot
        public static final double kTrackWidth = 0.61;
        // Distance between front and back wheels on robot
        public static final double kWheelBase = 0.61;

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
    }

    public static final class ModuleConstants {
        public static final double kMaxModuleAngularSpeedRadiansPerSecond = 16 * Math.PI; //Limits for wheel turning profiled PID
        public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 16 * Math.PI;

        public static final int kEncoderCPR = 4096;
        public static final double kWheelDiameterMeters = 0.15;
        public static final double kDriveEncoderDistancePerPulse =
                // Assumes the encoders are directly mounted on the wheel shafts
                (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;
        public static final double kDriveEncoderScale = 0.0470915; //0.0470915
 
        public static final double kTurningEncoderDistancePerPulse =
                // Assumes the encoders are on a 1:1 reduction with the module shaft.
                (2 * Math.PI) / (double) kEncoderCPR;

        public static double kPModuleTurningController = -1.1; //PID Values for angular wheel rotation
        public static double kIModuleTurningController = 0;
        public static double kDModuleTurningController = -0.01; //0.02

        public static final double kPModuleDriveController = 1; //P 0 0 for drive controller of swerve module
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 10; //Max speed of auton - 2.3 - 1.625
        public static final double kMaxAccelerationMetersPerSecondSquared = 1.5; //Max acceleration of auton
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI; //Max rotational speed of auton
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI; //Max rotational acceleration of auton

        public static final double kPXController = 3.05; // P 0 0 values for auton X, Y, and Theta controllers
        public static final double kPYController = 3.05;
        public static final double kDXController = 0.083; //0.093 // P 0 0 values for auton X, Y, and Theta controllers
        public static final double kDYController = 0.083;
        public static final double kPThetaController = 0.78;
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
              new Rotation3d())
        );
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
        public static final String Limelight1 = "limelight";

    }

    public static final class GamepadConstants {
        // deadzone
        public static final double kDeadZone = 0.15;

        // Primary Driver Controller Port Number.
        public static final int kPrimaryDriver = RobotMap.kPrimaryControllerPort;

        // Secondary Driver Controller Port Number.
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
        public static final int mech2dAdjustment = 45;
        public static final double limitOffset = -135;

        // misc.
        public static final double initiaLifterValue = 212;
        public static final double initialFirstExtenderValue = 0;
        public static final double initialSecondExtenderValue = 0;

        public static final double groundLifterValue = 94.5;
        public static final double groundFirstExtenderValue = 0;
        public static final double groundSecondExtenderValue = 0;

        public static final double balancingLifterValue = 115;
        public static final double balancingFirstExtenderValue = 0;
        public static final double balancingSecondExtenderValue = 0;

        public static final double humanLifterValue = 173.5; //171.5
        public static final double humanFirstExtenderValue = 60 * armEncoderCPR;
        public static final double humanSecondExtenderValue = 0 * armEncoderCPR;

        public static final double sideStationLifterValue = 167.9;
        public static final double sideStationFirstExtenderValue = 12 * armEncoderCPR;
        public static final double sideStationSecondExtenderValue = 0 * armEncoderCPR;

        public static final double retractLifterValue = 190;
        public static final double retractFirstExtenderValue = 0 * armEncoderCPR;
        public static final double retractSecondExtenderValue = 0 * armEncoderCPR;

        // cube
        public static final double midCubeLifterValue = 155.7;
        public static final double midCubeFirstExtenderValue = 39 * armEncoderCPR;
        public static final double midCubeSecondExtenderValue = 0 * armEncoderCPR;

        public static final double highCubeLifterValue = 171;
        public static final double highCubeFirstExtenderValue = 55.7 * armEncoderCPR;
        public static final double highCubeSecondExtenderValue = 58.9 * armEncoderCPR;
        
        // cone
        public static final double midConeLifterValue = 177;
        public static final double midConeFirstExtenderValue = 60 * armEncoderCPR;
        public static final double midConeSecondExtenderValue = 0 * armEncoderCPR;

        public static final double highConeLifterValue = 173;
        public static final double highConeFirstExtenderValue = 60 * armEncoderCPR;
        public static final double highConeSecondExtenderValue = 64.5 * armEncoderCPR;

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
        public static final State balancingState = new State(balancingLifterValue, balancingFirstExtenderValue, balancingSecondExtenderValue);
        public static final State humanStationState = new State(humanLifterValue, humanFirstExtenderValue, humanSecondExtenderValue);
        public static final State sideStationState = new State(sideStationLifterValue, sideStationFirstExtenderValue, sideStationSecondExtenderValue);
        public static final State retractState = new State(retractLifterValue, retractFirstExtenderValue, retractSecondExtenderValue);

        //PID values
        public static final double lifterP = 0.015; //0.015
        public static final double lifterI = 0.0015; //0.0015
        public static final double lifterD = 0.0014; //0.002

        public static final double firstExtenderP = 0.0001;
        public static final double firstExtenderI = 0;
        public static final double firstExtenderD = 0.000001;

        public static final double secondExtenderP = 0.00004;
        public static final double secondExtenderI = 0;
        public static final double secondExtenderD = 0.0000027;

        //general
        public static final double firstExtenderMaxExtend = 60.25 * armEncoderCPR; //60.25
        public static final double secondExtenderMaxExtend = 70 * armEncoderCPR;
        public static final double lifterSpeed = 1; //0.65
        public static final double manualLifterSpeed = 0.18;
        public static final double extenderSpeed = 0.8; //0.525
        public static final double linearExtenderSpeed = 0.24;

        public static final double horizontalMaxEncoderValue = 33.7 * armEncoderCPR;
        public static final double veritcalMaxEncoderValue = 44.2 * armEncoderCPR;
    }    
}
