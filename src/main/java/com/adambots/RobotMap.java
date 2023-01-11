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
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

/**
 * Define all the devices here
 */
public class RobotMap {
    // public static final Solenoid CatapultStop = new Solenoid(PneumaticsModuleType.CTREPCM,
    //         Constants.RAISE_CATAPULT_STOP_SOL_PORT);

    // public static final Solenoid BlasterHood = new Solenoid(PneumaticsModuleType.CTREPCM,
    //         Constants.RAISE_BLASTER_HOOD_SOL_PORT);
    // public static final DoubleSolenoid RungClamp = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,
    //         Constants.RAISE_HANG_CLAMP_SOL_PORT, Constants.LOWER_HANG_CLAMP_SOL_PORT);

    // public static final Lidar LidarSensor = Lidar.getInstance();

    // public static final ColorSensor ColorSensor = new ColorSensor();

    // public static final DigitalInput bandHomeSwitch = new DigitalInput(Constants.BAND_HOME_LIMIT_SWITCH_PORT);
    // public static final PhotoEye rungArmAdvancedSwitch = new PhotoEye(Constants.RUNG_ARM_ADVANCED_PHOTO_EYE_PORT);
    // public static final Solenoid YellowLight = new Solenoid(PneumaticsModuleType.CTREPCM,
    //         Constants.YELLOW_LEDS_SOL_PORT);

    // public static final PhotoEye IntakePhotoEye = new PhotoEye(7);
    // public static final PhotoEye SpacingPhotoEye = new PhotoEye(8);
    // public static final PhotoEye ExitPhotoEye = new PhotoEye(9);

    // public static final Counter IntakeCounter = new Counter(IntakePhotoEye.getDigitalInput());
    // public static final Counter ExitCounter = new Counter(ExitPhotoEye.getDigitalInput());

    public static final Gyro GyroSensor = Gyro.getInstance();
    public static final Solenoid RingLight = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.kRingLightPort);

    // Robot swerve modules
    public static final SwerveModule frontLeftSwerveModule = new SwerveModule(
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftTurningMotorPort,
            DriveConstants.kFrontLeftEncoderPort,
            DriveConstants.kFrontLeftDriveEncoderReversed,
            DriveConstants.kFrontLeftTurningEncoderReversed);

    public static final SwerveModule rearLeftSwerveModule = new SwerveModule(
            DriveConstants.kRearLeftDriveMotorPort,
            DriveConstants.kRearLeftTurningMotorPort,
            DriveConstants.kRearLeftEncoderPort,
            DriveConstants.kRearLeftDriveEncoderReversed,
            DriveConstants.kRearLeftTurningEncoderReversed);

    public static final SwerveModule frontRightSwerveModule = new SwerveModule(
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightEncoderPort,
            DriveConstants.kFrontRightDriveEncoderReversed,
            DriveConstants.kFrontRightTurningEncoderReversed);

    public static final SwerveModule rearRightSwerveModule = new SwerveModule(
            DriveConstants.kRearRightDriveMotorPort,
            DriveConstants.kRearRightTurningMotorPort,
            DriveConstants.kRearRightEncoderPort,
            DriveConstants.kRearRightDriveEncoderReversed,
            DriveConstants.kRearRightTurningEncoderReversed);
}
