// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.commands.autonCommands;

import java.util.List;

import com.adambots.Constants;
import com.adambots.Constants.AutoConstants;
import com.adambots.Constants.DriveConstants;
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
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class DriveToAprilTagCommand extends CommandBase {
  
  private DrivetrainSubsystem driveTrainSubsystem;
  private Pose2d field;
  private int index;
  private Gyro gyro;
  private SwerveControllerCommand swerveControllerCommand;

  public DriveToAprilTagCommand(DrivetrainSubsystem driveTrainSubsystem, Pose2d field, int index, Gyro gyro) {
    addRequirements(driveTrainSubsystem);

    this.driveTrainSubsystem = driveTrainSubsystem;
    this.field = field;
    this.index = index;
    this.gyro = gyro;
  }


  @Override
  public void initialize() {
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        .setKinematics(DriveConstants.kDriveKinematics);

      field = new Pose2d(field.getX(), field.getY(), gyro.getRotation2d());

    Pose2d aprilTag = Constants.AutoConstants.aprilTags.get(index).toPose2d();
    Translation2d waypoint = new Translation2d();
    if (index <= 4) { //Check if apriltag is on left or right side of the field to get a waypoint in front of it and modify the rotation of the apriltag pose
      waypoint = new Translation2d(aprilTag.getX()-0.85, aprilTag.getY());
      aprilTag = new Pose2d(aprilTag.getX()-0.8, aprilTag.getY(), new Rotation2d(0));
    } else {
      waypoint = new Translation2d(aprilTag.getX()+0.85, aprilTag.getY());
      aprilTag = new Pose2d(aprilTag.getX()+0.8, aprilTag.getY(), new Rotation2d(Units.degreesToRadians(180)));
      //0.6
    }

    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        field,
        List.of(waypoint),
        aprilTag,
        config); 

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, AutoConstants.kDThetaController, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    swerveControllerCommand = new SwerveControllerCommand(
      trajectory,
      driveTrainSubsystem::getPose, // Functional interface to feed supplier
      DriveConstants.kDriveKinematics,

      // Position controllers
      new PIDController(AutoConstants.kPXController, 0, AutoConstants.kDXController),
      new PIDController(AutoConstants.kPYController, 0, AutoConstants.kDYController),
      thetaController,
      driveTrainSubsystem::setModuleStates,
      driveTrainSubsystem);

    // Reset odometry to the starting pose of the trajectory.
    driveTrainSubsystem.resetOdometry(trajectory.getInitialPose());

    Constants.DriveConstants.field.getObject("traj").setTrajectory(trajectory);
    
    // Run path following command, then stop at the end.
    swerveControllerCommand.andThen(() -> driveTrainSubsystem.stop()).schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true; //we can instantly return true since this command just serves to schedule a different command
  }
}
