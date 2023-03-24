// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.commands.autonCommands;

import java.util.List;

import com.adambots.Constants;
import com.adambots.Constants.AutoConstants;
import com.adambots.Constants.DriveConstants;
import com.adambots.Vision.VisionHelpers;
import com.adambots.sensors.Gyro;
import com.adambots.subsystems.DrivetrainSubsystem;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class DriveToTowerCommand extends CommandBase {
  
  private DrivetrainSubsystem driveTrainSubsystem;
  private Gyro gyro;

  ProfiledPIDController thetaController;
  PIDController xController;
  PIDController yController;

  public DriveToTowerCommand(DrivetrainSubsystem driveTrainSubsystem, Gyro gyro) {
    addRequirements(driveTrainSubsystem);

    this.driveTrainSubsystem = driveTrainSubsystem;
    this.gyro = gyro;
  }


  @Override
  public void initialize() {

    thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, AutoConstants.kDThetaController, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    xController = new PIDController(AutoConstants.kPXController, 0, AutoConstants.kDXController);

    xController.setSetpoint(0);
    thetaController.setGoal(90);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    Pose2d robotPos = driveTrainSubsystem.getPose();
    driveTrainSubsystem.resetOdometry(robotPos);

    // var dX = VisionHelpers.getDistanceAway();

    // var dist = Math.sqrt(dX*dX + dY*dY);

   // var forwardCommand = Math.max(Math.min(xController.calculate(dist), 0.06), -0.06);
    var sidewaysCommand = Math.max(Math.min(yController.calculate(10), 0.06), -0.06);
    var thetaCommand = thetaController.calculate(robotPos.getRotation().getDegrees());

    driveTrainSubsystem.drive(0, -sidewaysCommand, Math.max(Math.min(thetaCommand, 0.1), -0.1), false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrainSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // var dist = Math.sqrt(index)
    return false;
  }
}
