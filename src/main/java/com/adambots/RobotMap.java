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
        // Solenoid(PneumaticsModuleType.CTREPCM, Constants.YELLOW_LEDS_SOL_PORT);

        // public static final PhotoEye IntakePhotoEye = new PhotoEye(7);

        // public static final Counter IntakeCounter = new Counter(IntakePhotoEye.getDigitalInput());

        // PORTS Definition - This should be the only place to define all ports
        public final static I2C.Port I2C_PORT = I2C.Port.kOnboard;
        public static final int kLidarDio = 5; // Not used
        public static final int kRingLightPort = 4;
        
        // CAN bus ports
        public static final int kRearLeftEncoderPort = 2;
        public static final int kRearRightEncoderPort = 3;
        public static final int kFrontLeftEncoderPort = 4;
        public static final int kFrontRightEncoderPort = 5;
        public static final int kFrontRightTurningMotorPort = 11;
        public static final int kFrontRightDriveMotorPort = 12;
        public static final int kRearRightTurningMotorPort = 13;
        public static final int kRearRightDriveMotorPort = 14;
        public static final int kRearLeftTurningMotorPort = 15;
        public static final int kRearLeftDriveMotorPort = 16;
        public static final int kFrontLeftTurningMotorPort = 17;
        public static final int kFrontLeftDriveMotorPort = 18;
        
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
}
