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
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.WPI_CANCoder;

public class SwerveModule {
  private final CANSparkMax m_driveMotor;
  private final CANSparkMax m_turningMotor;

  private final WPI_CANCoder m_encoder;
  private final CANCoderConfiguration m_canCoderConfig = new CANCoderConfiguration();

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
      int encoderChannel,
      boolean driveEncoderReversed,
      boolean turningEncoderReversed) {
    // m_driveMotor = new Spark(driveMotorChannel);
    m_driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
    m_turningMotor = new CANSparkMax(turningMotorChannel, MotorType.kBrushless);
    // m_driveMotor.setIdleMode(IdleMode.kBrake);
    // m_turningMotor.setIdleMode(IdleMode.kBrake);

    m_encoder = new WPI_CANCoder(encoderChannel);
    m_canCoderConfig.unitString = "rad";
    // m_encoder.configAllSettings(m_canCoderConfig);
    m_encoder.clearStickyFaults();

    // m_driveEncoder = new Encoder(driveEncoderChannels[0], driveEncoderChannels[1]);

    // m_turningEncoder = new Encoder(turningEncoderChannels[0], turningEncoderChannels[1]);

    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    // m_driveEncoder.setDistancePerPulse(ModuleConstants.kDriveEncoderDistancePerPulse);

    // Set whether drive encoder should be reversed or not
    // m_driveEncoder.setReverseDirection(driveEncoderReversed);

    // Set the distance (in this case, angle) per pulse for the turning encoder.
    // This is the the angle through an entire rotation (2 * pi) divided by the
    // encoder resolution.
    // m_turningEncoder.setDistancePerPulse(ModuleConstants.kTurningEncoderDistancePerPulse);

    // Set whether turning encoder should be reversed or not
    // m_turningEncoder.setReverseDirection(turningEncoderReversed);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    // m_encoder.
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
    double speedMetersPerSecond = m_driveMotor.getEncoder().getVelocity() / 60.0;
    double turningRadians = Units.degreesToRadians(m_encoder.getAbsolutePosition()); //assuming that setting the cancoder config to rad will return radians. if not, convert.
    return new SwerveModuleState(speedMetersPerSecond, new Rotation2d(turningRadians));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    double distance = m_driveMotor.getEncoder().getPosition() * ModuleConstants.kDriveEncoderDistancePerPulse;
    double turningDistance = m_encoder.getAbsolutePosition() * ModuleConstants.kTurningEncoderDistancePerPulse;
    return new SwerveModulePosition(
        distance, new Rotation2d(turningDistance));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {

    double speedMetersPerSecond = ModuleConstants.kDriveEncoderDistancePerPulse * m_encoder.getVelocity();
    double turningRadians = Units.degreesToRadians(m_encoder.getAbsolutePosition()); //assuming that setting the cancoder config to rad will return radians. if not, convert.

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
    m_driveMotor.getEncoder().setPosition(0);
    m_encoder.setPosition(0);
  }
}