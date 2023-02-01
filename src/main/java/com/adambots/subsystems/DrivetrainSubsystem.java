// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.subsystems;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import java.util.HashMap;
import java.util.Map;

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
  private final SwerveModule m_frontLeft;
  private final SwerveModule m_rearLeft;
  private final SwerveModule m_frontRight;
  private final SwerveModule m_rearRight;

  // The gyro sensor
  private final Gyro m_gyro;

  // Odometry class for tracking robot pose
  private SwerveDriveOdometry m_odometry;

  // Field details that can be viewed in Glass
  private final Field2d m_field = new Field2d();
  private SimDouble m_simAngle;
  HashMap<ModulePosition, SwerveModule> swerveModules;

  public DrivetrainSubsystem(HashMap<ModulePosition, SwerveModule> modules, Gyro gyro){

    this(
      modules.get(DriveConstants.ModulePosition.FRONT_LEFT),
      modules.get(DriveConstants.ModulePosition.FRONT_RIGHT),
      modules.get(DriveConstants.ModulePosition.REAR_LEFT),
      modules.get(DriveConstants.ModulePosition.REAR_RIGHT),
      gyro
      );

      this.swerveModules = modules;
  }

  private DrivetrainSubsystem(SwerveModule frontLeft, SwerveModule rearLeft, SwerveModule frontRight,
      SwerveModule rearRight, Gyro gyro) {
    m_frontLeft = frontLeft;
    m_frontRight = frontRight;
    m_rearLeft = rearLeft;
    m_rearRight = rearRight;
    m_gyro = gyro;

    m_odometry = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
            m_frontLeft.getM_position(),
            m_frontRight.getM_position(),
            m_rearLeft.getM_position(),
            m_rearRight.getM_position()
        });

    SmartDashboard.putData("Field", m_field);

    // When the robot starts, the Gyro may not be ready to reset - wait 1 second and then reset
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
        // m_frontLeft.getState(),
        // m_frontRight.getState(),
        // m_rearLeft.getState(),
        // m_rearRight.getState(),
        new SwerveModulePosition[] {
            m_frontLeft.getM_position(),
            m_frontRight.getM_position(),
            m_rearLeft.getM_position(),
            m_rearRight.getM_position()
        });

    SmartDashboard.putNumber("m_frontLeft", m_frontLeft.getState().angle.getDegrees());
    SmartDashboard.putNumber("m_rearLeft", m_rearLeft.getState().angle.getDegrees());
    SmartDashboard.putNumber("m_frontRight", m_frontRight.getState().angle.getDegrees());
    SmartDashboard.putNumber("m_rearRight", m_rearRight.getState().angle.getDegrees());

    SmartDashboard.putNumber("gyro", getHeading());

    SmartDashboard.putNumber("m_frontLeft speeed", m_frontLeft.getState().speedMetersPerSecond);
    SmartDashboard.putNumber("m_rearLeft speeed", m_rearLeft.getState().speedMetersPerSecond);
    SmartDashboard.putNumber("m_frontRight speeed", m_frontRight.getState().speedMetersPerSecond);
    SmartDashboard.putNumber("m_rearRight speeed", m_rearRight.getState().speedMetersPerSecond);

    SmartDashboard.putNumber("Odom Rot", m_gyro.getRotation2d().getDegrees());
    SmartDashboard.putNumber("Odom X", m_odometry.getPoseMeters().getX());
    SmartDashboard.putNumber("Odom Y", m_odometry.getPoseMeters().getY());

    m_field.setRobotPose(getPose());

    // SmartDashboard.putNumber("getTroddle", ex3dPro.getThrottle());
    // SmartDashboard.putNumber("getTwist", ex3dPro.getTwist());
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
        // m_frontLeft.getState(),
        // m_frontRight.getState(),
        // m_rearLeft.getState(),
        // m_rearRight.getState(),
        new SwerveModulePosition[] {
            m_frontLeft.getM_position(),
            m_frontRight.getM_position(),
            m_rearLeft.getM_position(),
            m_rearRight.getM_position()
        });

    System.out.println("Pose: " + getPose().toString());

    m_field.setRobotPose(getPose());

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
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
            m_frontLeft.getM_position(),
            m_frontRight.getM_position(),
            m_rearLeft.getM_position(),
            m_rearRight.getM_position()
        },
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            // ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot,
            // m_gyro.getRotation2d())
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d())

            : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);

    // System.out.printf("XSpeed: %f, YSpeed: %f, Rot: %f\n", xSpeed, ySpeed, rot);
    // System.out.printf("Left Speed: %f, Rot: %f\n",
    // swerveModuleStates[0].speedMetersPerSecond,
    // swerveModuleStates[0].angle.getDegrees());
    /**
     */
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);

    // SmartDashboard.putNumber("m_frontLeftTarget",
    // swerveModuleStates[0].angle.getDegrees());
    // SmartDashboard.putNumber("m_frontRightTarget",
    // swerveModuleStates[1].angle.getDegrees());
    // SmartDashboard.putNumber("m_rearLeftTarget",
    // swerveModuleStates[2].angle.getDegrees());
    // SmartDashboard.putNumber("m_rearRightTarget",
    // swerveModuleStates[3].angle.getDegrees());

  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

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
    // return m_gyro.getRotation2d().getDegrees();
    return Math.IEEEremainder(m_gyro.getAngle(), 360); // Constrain to 360
    // return MathUtil.inputModulus(m_gyro.getAngle(), -180, 180); // use this if you want to constrain by -180 to 180
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
    m_frontLeft.setPIDValues(kP, kI, kD);
    m_frontRight.setPIDValues(kP, kI, kD);
    m_rearLeft.setPIDValues(kP, kI, kD);
    m_rearRight.setPIDValues(kP, kI, kD);
  }

  public void stop() {
    drive(0, 0, 0, false);
  }

  public void hockeyStop(){
    // TODO: Set proper angles 

    m_frontLeft.turn(45);
    m_frontRight.turn(-45);
    m_rearLeft.turn(-45);
    m_rearRight.turn(45);
  }
}