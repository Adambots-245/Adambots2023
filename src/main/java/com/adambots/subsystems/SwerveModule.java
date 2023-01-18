// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import com.adambots.Constants.ModuleConstants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.WPI_CANCoder;

public class SwerveModule {
  private final CANSparkMax m_driveMotor;
  private final CANSparkMax m_turningMotor;

  private final WPI_CANCoder m_turnEncoder;
  private final RelativeEncoder m_driveEncoder;
  // private final CANCoderConfiguration m_canCoderConfig = new CANCoderConfiguration();s

  private final PIDController m_drivePIDController =
      new PIDController(ModuleConstants.kPModuleDriveController, 0, 0);

  // Using a TrapezoidProfile PIDController to allow for smooth turning
  private final ProfiledPIDController m_turningPIDController =
      new ProfiledPIDController(
          ModuleConstants.kPModuleTurningController,
          ModuleConstants.kIModuleTurningController,
          ModuleConstants.kDModuleTurningController,
          new TrapezoidProfile.Constraints(
              ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond,
              ModuleConstants.kMaxModuleAngularAccelerationRadiansPerSecondSquared));

  public void setPIDValues(double kP, double kI, double kD) {
    m_turningPIDController.setP(kP);
    m_turningPIDController.setI(kI);
    m_turningPIDController.setD(kD);
  }

  /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorChannel The channel of the drive motor.
   * @param turningMotorChannel The channel of the turning motor.
   * @param driveEncoderChannels The channels of the drive encoder.
   * @param turningEncoderChannels The channels of the turning encoder.
   * @param driveEncoderReversed Whether the drive encoder is reversed.
   * @param turningEncoderReversed Whether the turning encoder is reversed.
   */
  public SwerveModule(
      int driveMotorChannel,
      int turningMotorChannel,
      int turningEncoderChannel,
      boolean driveEncoderReversed,
      boolean turningEncoderReversed) {
    
    m_driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
    m_turningMotor = new CANSparkMax(turningMotorChannel, MotorType.kBrushless);
    // m_driveMotor.setIdleMode(IdleMode.kBrake);
    // m_turningMotor.setIdleMode(IdleMode.kBrake);

    m_turnEncoder = new WPI_CANCoder(turningEncoderChannel);
    m_driveEncoder = m_driveMotor.getEncoder();
    // m_canCoderConfig.unitString = "rad";
    // m_encoder.configAllSettings(m_canCoderConfig);
    m_turnEncoder.clearStickyFaults();

    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // return new SwerveModuleState(m_driveEncoder.getRate(), new Rotation2d(m_turningEncoder.get()));
    // double speedMetersPerSecond = ModuleConstants.kDriveEncoderDistancePerPulse * m_encoder.getVelocity();
    double speedMetersPerSecond = m_driveEncoder.getVelocity() / 60.0;
    double turningRadians = Units.degreesToRadians(m_turnEncoder.getAbsolutePosition()); //assuming that setting the cancoder config to rad will return radians. if not, convert.
    return new SwerveModuleState(speedMetersPerSecond, new Rotation2d(turningRadians));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    double distance = m_driveEncoder.getPosition() * ModuleConstants.kDriveEncoderDistancePerPulse;
    double turningDistance = m_turnEncoder.getAbsolutePosition() * ModuleConstants.kTurningEncoderDistancePerPulse;
    return new SwerveModulePosition(
        distance, new Rotation2d(turningDistance));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {

    double speedMetersPerSecond = ModuleConstants.kDriveEncoderDistancePerPulse * m_turnEncoder.getVelocity();
    double turningRadians = Units.degreesToRadians(m_turnEncoder.getAbsolutePosition()); //assuming that setting the cancoder config to rad will return radians. if not, convert.

    // System.out.printf("Speed: %f, Turn: %f\n", speedMetersPerSecond, turningRadians);
    
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state =
        SwerveModuleState.optimize(desiredState, new Rotation2d(turningRadians));

    // Calculate the drive output from the drive PID controller.
    final double driveOutput =
        m_drivePIDController.calculate(speedMetersPerSecond, state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput =
        m_turningPIDController.calculate(turningRadians, state.angle.getRadians());

    // Calculate the turning motor output from the turning PID controller.

    m_driveMotor.set(driveOutput);
    m_turningMotor.set(turnOutput);

    // System.out.printf("Drive Output: %f\n", driveOutput);
    // System.out.printf("Turn Output: %f\n", turnOutput);
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_driveEncoder.setPosition(0);
    m_turnEncoder.setPosition(0);
  }
}