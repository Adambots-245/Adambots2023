/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.adambots;

import com.adambots.Constants.DriveConstants;
import com.adambots.sensors.ColorSensor;
import com.adambots.sensors.Gyro;
import com.adambots.sensors.Lidar;
import com.adambots.sensors.PhotoEye;
import com.adambots.subsystems.SwerveModule;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

/**
 * Define all the devices here
 */
public class RobotMap {
        // public static final Solenoid CatapultStop = new
        // Solenoid(PneumaticsModuleType.CTREPCM,
        // Constants.RAISE_CATAPULT_STOP_SOL_PORT);

        // public static final Solenoid BlasterHood = new
        // Solenoid(PneumaticsModuleType.CTREPCM,
        // Constants.RAISE_BLASTER_HOOD_SOL_PORT);
        // public static final DoubleSolenoid RungClamp = new
        // DoubleSolenoid(PneumaticsModuleType.CTREPCM,
        // Constants.RAISE_HANG_CLAMP_SOL_PORT, Constants.LOWER_HANG_CLAMP_SOL_PORT);

        // public static final Lidar LidarSensor = Lidar.getInstance();

        // public static final ColorSensor ColorSensor = new ColorSensor();

        // public static final DigitalInput bandHomeSwitch = new
        // DigitalInput(Constants.BAND_HOME_LIMIT_SWITCH_PORT);
        // public static final PhotoEye rungArmAdvancedSwitch = new
        // PhotoEye(Constants.RUNG_ARM_ADVANCED_PHOTO_EYE_PORT);
        // public static final Solenoid YellowLight = new
        // Solenoid(PneumaticsModuleType.CTREPCM,
        // Constants.YELLOW_LEDS_SOL_PORT);

        // public static final PhotoEye IntakePhotoEye = new PhotoEye(7);
        // public static final PhotoEye SpacingPhotoEye = new PhotoEye(8);
        // public static final PhotoEye ExitPhotoEye = new PhotoEye(9);

        // public static final Counter IntakeCounter = new
        // Counter(IntakePhotoEye.getDigitalInput());
        // public static final Counter ExitCounter = new
        // Counter(ExitPhotoEye.getDigitalInput());

        // PORTS Definition - This should be the only place to define all ports
        public final static I2C.Port I2C_PORT = I2C.Port.kOnboard;
        public static final int kLidarDio = 5; // Not used
        public static final int kRingLightPort = 4;
        
        // CAN bus ports
        public static final int kRearLeftEncoderPort = 2;
        public static final int kRearRightEncoderPort = 3;
        public static final int kFrontLeftEncoderPort = 4;
        public static final int kFrontRightEncoderPort = 5;
        public static final int kFrontRightTurningMotorPort = 13;
        public static final int kFrontRightDriveMotorPort = 11;
        public static final int kRearRightTurningMotorPort = 18;
        public static final int kRearRightDriveMotorPort = 16;
        public static final int kRearLeftTurningMotorPort = 14;
        public static final int kRearLeftDriveMotorPort = 12;
        public static final int kFrontLeftTurningMotorPort = 15;
        public static final int kFrontLeftDriveMotorPort = 17;

        // Arm and Grabby ports
        public static final int armLiftPort = 5;
        public static final int leftArmExtenderPort = 6;
        public static final int rightArmExtenderPort = 7;
        public static final int grabbyPort = 1;
        public static final int ungrabbyPort = 0;
        // public static final int rightGrabbyPort = 0;
        public static final int leftArmLimitPort = 0;
        public static final int rightArmLimitPort = 1;
        public static final int armRotationEncoderPort = 8;

        // Arm and Grabby Devices
        public static final DoubleSolenoid grabby = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, grabbyPort, ungrabbyPort);
        // public static final Solenoid rightGrabby = new Solenoida (PneumaticsModuleType.CTREPCM, rightGrabbyPort);
        
        public static final WPI_TalonFX leftArmExtender = new WPI_TalonFX(leftArmExtenderPort);
        public static final WPI_TalonFX rightArmExtender = new WPI_TalonFX(rightArmExtenderPort);
        public static final WPI_TalonFX armLifter = new WPI_TalonFX(armLiftPort);

        public static final CANCoder armRotationEncoder = new CANCoder(armRotationEncoderPort);        
        public static final PhotoEye leftArmPhotoEye = new PhotoEye(leftArmLimitPort);
        public static final PhotoEye rightArmPhotoEye = new PhotoEye(rightArmLimitPort);



        // Operator Interface (Joystick and XBoxControllers)
        public static final int kJoystickControllerPort = 0;
        public static final int kPrimaryControllerPort = 1; // XBOX Controller
        public static final int kSecondaryControllerPort = 2; // XBOX Controller

        // ROBOT Devices and Sensors
        public static final Gyro GyroSensor = Gyro.getInstance();
        public static final Solenoid RingLight = new Solenoid(PneumaticsModuleType.CTREPCM, RobotMap.kRingLightPort);

        // Robot swerve modules
        public static final SwerveModule frontLeftSwerveModule = new SwerveModule(
                        RobotMap.kFrontLeftDriveMotorPort,
                        RobotMap.kFrontLeftTurningMotorPort,
                        RobotMap.kFrontLeftEncoderPort,
                        DriveConstants.kFrontLeftDriveEncoderReversed,
                        DriveConstants.kFrontLeftTurningEncoderReversed);

        public static final SwerveModule rearLeftSwerveModule = new SwerveModule(
                        RobotMap.kRearLeftDriveMotorPort,
                        RobotMap.kRearLeftTurningMotorPort,
                        RobotMap.kRearLeftEncoderPort,
                        DriveConstants.kRearLeftDriveEncoderReversed,
                        DriveConstants.kRearLeftTurningEncoderReversed);

        public static final SwerveModule frontRightSwerveModule = new SwerveModule(
                        RobotMap.kFrontRightDriveMotorPort,
                        RobotMap.kFrontRightTurningMotorPort,
                        RobotMap.kFrontRightEncoderPort,
                        DriveConstants.kFrontRightDriveEncoderReversed,
                        DriveConstants.kFrontRightTurningEncoderReversed);

        public static final SwerveModule rearRightSwerveModule = new SwerveModule(
                        RobotMap.kRearRightDriveMotorPort,
                        RobotMap.kRearRightTurningMotorPort,
                        RobotMap.kRearRightEncoderPort,
                        DriveConstants.kRearRightDriveEncoderReversed,
                        DriveConstants.kRearRightTurningEncoderReversed);
        // Swerve Drive Constants
}
