// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.commands.autonCommands;

import com.adambots.Constants;
import com.adambots.Constants.AutoConstants;
import com.adambots.Vision.VisionHelpers;
import com.adambots.sensors.Gyro;
import com.adambots.subsystems.DrivetrainSubsystem;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TestDriveToAprilTagCommand extends CommandBase {
  
  private DrivetrainSubsystem driveTrainSubsystem;
  private int index;
  private Gyro gyro;
  private Pose2d aprilTagPos;

  ProfiledPIDController thetaController;
  PIDController xController;
  PIDController yController;

  public TestDriveToAprilTagCommand(DrivetrainSubsystem driveTrainSubsystem, int index, Gyro gyro) {
    addRequirements(driveTrainSubsystem);

    this.driveTrainSubsystem = driveTrainSubsystem;
    this.index = index;
    this.gyro = gyro;
  }


  @Override
  public void initialize() {
    aprilTagPos = Constants.AutoConstants.aprilTags.get(index).toPose2d();

    var finalOffset = 0.65;
    if (index <= 4) { //Check if apriltag is on left or right side of the field to get a waypoint in front of it and modify the rotation of the apriltag pose
      aprilTagPos = new Pose2d(aprilTagPos.getX()-finalOffset, aprilTagPos.getY(), new Rotation2d(0));
    } else {
      aprilTagPos = new Pose2d(aprilTagPos.getX()+finalOffset, aprilTagPos.getY(), new Rotation2d(Units.degreesToRadians(180)));
    }

    thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, AutoConstants.kDThetaController, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    xController = new PIDController(AutoConstants.kPXController, 0, AutoConstants.kDXController);
    yController = new PIDController(AutoConstants.kPYController, 0, AutoConstants.kDYController);

    xController.setSetpoint(0);
    yController.setSetpoint(0);
    thetaController.setGoal(aprilTagPos.getRotation().getDegrees());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    Pose2d robotPos = VisionHelpers.getAprilTagPose2d();
    driveTrainSubsystem.resetOdometry(robotPos);

    var dX = VisionHelpers.getTX();
    var dY = VisionHelpers.getLeftRightDistance();

    var dist = Math.sqrt(dX*dX + dY*dY);

    var forwardCommand = Math.max(Math.min(xController.calculate(dist), 0.06), -0.06);
    var sidewaysCommand = Math.max(Math.min(yController.calculate(dY), 0.06), -0.06);
    var thetaCommand = thetaController.calculate(robotPos.getRotation().getDegrees());

    driveTrainSubsystem.drive(-forwardCommand, -sidewaysCommand, Math.max(Math.min(thetaCommand, 0.1), -0.1), false);
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
