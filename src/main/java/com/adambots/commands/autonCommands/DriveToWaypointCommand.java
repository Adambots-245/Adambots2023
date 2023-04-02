// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.commands.autonCommands;

import com.adambots.Constants.AutoConstants;
import com.adambots.sensors.Gyro;
import com.adambots.subsystems.DrivetrainSubsystem;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveToWaypointCommand extends CommandBase {
  private DrivetrainSubsystem drivetrainSubsystem;
  private PIDController xController;
  private PIDController yController;
  private PIDController thetaController;
  private Gyro gyro;
  private Pose2d waypoint;
  private double turnAtPercentAlongPath;
  private double initDist;
  private double inc;

  public DriveToWaypointCommand(DrivetrainSubsystem drivetrainSubsystem, Gyro gyro, Pose2d waypoint, double turnAtPercentAlongPath) {
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.gyro = gyro;
    this.waypoint = waypoint;
    this.turnAtPercentAlongPath = turnAtPercentAlongPath;

    xController = new PIDController(AutoConstants.kPXController, 0, AutoConstants.kDXController);
    yController = new PIDController(AutoConstants.kPYController, 0, AutoConstants.kDYController);

    thetaController = new PIDController(AutoConstants.kPThetaController, 0, AutoConstants.kDThetaController);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(drivetrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    xController.setSetpoint(waypoint.getX());
    yController.setSetpoint(waypoint.getY());
    thetaController.setSetpoint(Math.toRadians(gyro.getAngle()));

    initDist = getDist(drivetrainSubsystem.getPose(), waypoint);

    inc = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (1-(getDist(drivetrainSubsystem.getPose(), waypoint) / initDist) > turnAtPercentAlongPath) {
      thetaController.setSetpoint(waypoint.getRotation().getRadians());
    }

    double x = xController.calculate(drivetrainSubsystem.getPose().getX());
    double y = xController.calculate(drivetrainSubsystem.getPose().getY());
    double theta = thetaController.calculate(gyro.getAngle());

    drivetrainSubsystem.drive(x, y, theta, true);

    if (getDist(drivetrainSubsystem.getPose(), waypoint) < 0.3) {
      inc++;
    } else if (inc > 0) {
      inc--;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.stop();
  }

  public double getDist (Pose2d pos1, Pose2d pos2) {
    double x = Math.abs(pos1.getX() - pos2.getX());
    double y = Math.abs(pos1.getY() - pos2.getY());
        
    return Math.hypot(x, y);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return inc > 20;
  }
}
