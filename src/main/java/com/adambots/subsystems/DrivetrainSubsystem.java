// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.subsystems;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import java.util.HashMap;
import java.util.Map;

import com.adambots.Constants;
import com.adambots.Constants.DriveConstants;
import com.adambots.Constants.DriveConstants.ModulePosition;
import com.adambots.utils.ModuleMap;
import com.ctre.phoenix.unmanaged.Unmanaged;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DrivetrainSubsystem extends SubsystemBase {
  // Robot swerve modules
  // private final SwerveModule m_frontLeft;
  // private final SwerveModule m_rearLeft;
  // private final SwerveModule m_frontRight;
  // private final SwerveModule m_rearRight;

  // The gyro sensor
  private final Gyro m_gyro;

  // Odometry class for tracking robot pose
  private SwerveDriveOdometry m_odometry;

  // Field details that can be viewed in Glass
  private SimDouble m_simAngle;
  HashMap<ModulePosition, SwerveModule> swerveModules;

  public DrivetrainSubsystem(HashMap<ModulePosition, SwerveModule> modules, Gyro gyro) {

    // m_frontLeft = modules.get(DriveConstants.ModulePosition.FRONT_LEFT);
    // m_frontRight = modules.get(DriveConstants.ModulePosition.FRONT_RIGHT);
    // m_rearLeft = modules.get(DriveConstants.ModulePosition.REAR_LEFT);
    // m_rearRight = modules.get(DriveConstants.ModulePosition.REAR_RIGHT);

    this.swerveModules = modules;
    m_gyro = gyro;

    m_odometry = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, m_gyro.getRotation2d(),
        ModuleMap.orderedModulePositions(swerveModules)
    // new SwerveModulePosition[] {
    // m_frontLeft.getPosition(),
    // m_frontRight.getPosition(),
    // m_rearLeft.getPosition(),
    // m_rearRight.getPosition()
    // }
    );

    // When the robot starts, the Gyro may not be ready to reset - wait 1 second and
    // then reset
    new Thread(() -> {
      try {
        Thread.sleep(1000);
        zeroHeading();
      } catch (Exception e) {
      }
    }).start();

    if (RobotBase.isSimulation()) {

      var dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");

      m_simAngle = new SimDouble((SimDeviceDataJNI.getSimValueHandle(dev, "Yaw")));
    }
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        m_gyro.getRotation2d(),
        ModuleMap.orderedModulePositions(swerveModules)
    );

    Constants.DriveConstants.field.setRobotPose(getPose());
  }

  @Override
  public void simulationPeriodic() {
    ChassisSpeeds chassisSpeedSim = DriveConstants.kDriveKinematics.toChassisSpeeds(
        ModuleMap.orderedValues(getModuleStates(), new SwerveModuleState[0]));
    // want to simulate navX gyro changing as robot turns
    // information available is radians per second and this happens every 20ms
    // radians/2pi = 360 degrees so 1 degree per second is radians / 2pi
    // increment is made every 20 ms so radian adder would be (rads/sec) *(20/1000)
    // degree adder would be radian adder * 360/2pi
    // so degree increment multiplier is 360/100pi = 1.1459

    double temp = chassisSpeedSim.omegaRadiansPerSecond * 1.1459155;

    temp += m_simAngle.get();

    m_simAngle.set(temp);

    for (SwerveModule module : ModuleMap.orderedValuesList(swerveModules)) {
      module.simulationPeriodic();
    }

    m_odometry.update(
        new Rotation2d(m_simAngle.get()),
        ModuleMap.orderedModulePositions(swerveModules)

    // new SwerveModulePosition[] {
    // m_frontLeft.getPosition(),
    // m_frontRight.getPosition(),
    // m_rearLeft.getPosition(),
    // m_rearRight.getPosition()
    // }
    );

    // System.out.println("Pose: " + getPose().toString());

    Constants.DriveConstants.field.setRobotPose(getPose());

    Unmanaged.feedEnable(20);
  }

  public Map<ModulePosition, SwerveModuleState> getModuleStates() {
    Map<ModulePosition, SwerveModuleState> map = new HashMap<>();
    for (ModulePosition i : swerveModules.keySet()) {
      map.put(i, swerveModules.get(i).getState());
    }
    return map;
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose
   *          The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(m_gyro.getRotation2d(), ModuleMap.orderedModulePositions(swerveModules), pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed
   *          Speed of the robot in the x direction (forward).
   * @param ySpeed
   *          Speed of the robot in the y direction (sideways).
   * @param rot
   *          Angular rate of the robot.
   * @param fieldRelative
   *          Whether the provided x and y speeds are relative to the
   *          field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(-xSpeed, -ySpeed, rot, m_gyro.getRotation2d())

            : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);

    // System.out.printf("XSpeed: %f, YSpeed: %f, Rot: %f\n", xSpeed, ySpeed, rot);
    // System.out.printf("Left Speed: %f, Rot: %f\n",
    ModuleMap.setDesiredState(swerveModules, swerveModuleStates);
    // m_frontLeft.setDesiredState(swerveModuleStates[0]);
    // m_frontRight.setDesiredState(swerveModuleStates[1]);
    // m_rearLeft.setDesiredState(swerveModuleStates[2]);
    // m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates
   *          The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);

    ModuleMap.setDesiredState(swerveModules, desiredStates);

    // m_frontLeft.setDesiredState(desiredStates[0]);
    // m_frontRight.setDesiredState(desiredStates[1]);
    // m_rearLeft.setDesiredState(desiredStates[2]);
    // m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  // public void resetEncoders() {
  //   swerveModules.get(ModulePosition.FRONT_LEFT).resetEncoders();
  //   swerveModules.get(ModulePosition.FRONT_RIGHT).resetEncoders();
  //   swerveModules.get(ModulePosition.REAR_LEFT).resetEncoders();
  //   swerveModules.get(ModulePosition.REAR_RIGHT).resetEncoders();
  //   // m_frontLeft.resetEncoders();
  //   // m_frontRight.resetEncoders();
  //   // m_rearLeft.resetEncoders();
  //   // m_rearRight.resetEncoders();
  // }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from 0 to 360
   */
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
    // return Math.IEEEremainder(m_gyro.getAngle(), 360); // Constrain to 360
    // return MathUtil.inputModulus(m_gyro.getAngle(), -180, 180); // use this if
    // you want to constrain by -180 to 180
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public void setPIDValues(double kP, double kI, double kD) {
    swerveModules.get(ModulePosition.FRONT_LEFT).setPIDValues(kP, kI, kD);
    swerveModules.get(ModulePosition.FRONT_RIGHT).setPIDValues(kP, kI, kD);
    swerveModules.get(ModulePosition.REAR_LEFT).setPIDValues(kP, kI, kD);
    swerveModules.get(ModulePosition.REAR_RIGHT).setPIDValues(kP, kI, kD);
  }

  public void stop() {
    drive(0, 0, 0, false);
  }
}