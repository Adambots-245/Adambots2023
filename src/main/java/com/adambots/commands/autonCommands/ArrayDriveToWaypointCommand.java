// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.commands.autonCommands;

import com.adambots.Constants.AutoConstants;
import com.adambots.commands.autonCommands.AutonCommands.Waypoint;
import com.adambots.sensors.Gyro;
import com.adambots.subsystems.DrivetrainSubsystem;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ArrayDriveToWaypointCommand extends CommandBase {
  private DrivetrainSubsystem drivetrainSubsystem;
  private PIDController xController;
  private PIDController yController;
  private PIDController thetaController;
  private Gyro gyro;
  private Waypoint[] waypoints;
  private int index;
  private double startTime;
  private double finalInc;

  public ArrayDriveToWaypointCommand(DrivetrainSubsystem drivetrainSubsystem, Gyro gyro, Waypoint[] waypoints) {
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.gyro = gyro;
    this.waypoints = waypoints;

    xController = new PIDController(AutoConstants.kPXController, 0, AutoConstants.kDXController);
    yController = new PIDController(AutoConstants.kPYController, 0, AutoConstants.kDYController);

    thetaController = new PIDController(AutoConstants.kPThetaController, 0, AutoConstants.kDThetaController);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(drivetrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    index = 0;
    finalInc = 0;

    setTarget(index);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (index+1 == waypoints.length) {
      if (getDist(drivetrainSubsystem.getPose(), waypoints[index].pose()) < 0.3) {
        finalInc++;
      } else if (finalInc > 0) {
        finalInc--;
      }
    } else {
      if (System.currentTimeMillis() - startTime > waypoints[index].switchAtTime()*1000) {
        index++;
        setTarget(index);
      }
    }

    double x = xController.calculate(drivetrainSubsystem.getPose().getX());
    double y = xController.calculate(drivetrainSubsystem.getPose().getY());
    double theta = thetaController.calculate(gyro.getAngle());

    drivetrainSubsystem.drive(x, y, theta, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.stop();
  }

  public void setTarget (int i) {
    xController.setSetpoint(waypoints[i].pose().getX());
    yController.setSetpoint(waypoints[i].pose().getY());
    thetaController.setSetpoint(waypoints[i].pose().getRotation().getRadians());

    startTime = System.currentTimeMillis();
  }

  public double getDist (Pose2d pos1, Pose2d pos2) {
    double x = Math.abs(pos1.getX() - pos2.getX());
    double y = Math.abs(pos1.getY() - pos2.getY());
        
    return Math.hypot(x, y);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finalInc > 20;
  }
}
