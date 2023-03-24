package com.adambots.utils;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import com.adambots.Constants.AutoConstants;
import com.adambots.Constants.DriveConstants;
import com.adambots.subsystems.DrivetrainSubsystem;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class Functions {
    public static Trajectory getTrajectory (String filepath) {
        Trajectory trajectory = new Trajectory();

        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(filepath);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + filepath, ex.getStackTrace());
        }

        return trajectory;
    }

    public static SwerveControllerCommand CreateSwerveControllerCommand (DrivetrainSubsystem drivetrainSubsystem, Trajectory trajectory) {
        var thetaController = new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, AutoConstants.kDThetaController, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
            trajectory,
            drivetrainSubsystem::getPose, // Functional interface to feed supplier
            DriveConstants.kDriveKinematics,

            // Position controllers
            new PIDController(AutoConstants.kPXController, 0, AutoConstants.kDXController),
            new PIDController(AutoConstants.kPYController, 0, AutoConstants.kDYController),        
            thetaController,
            drivetrainSubsystem::setModuleStates,
            drivetrainSubsystem);

        // Run the "Glass" program and then choose NetworkTables -> SmartDashboard -> Field2d to view the Field.
        // The field image for 2023 is in utils folder
        // Constants.DriveConstants.field.getObject("traj").setTrajectory(trajectory);
        
        return swerveControllerCommand;
    }

    public static SwerveControllerCommand HumanSwerveControllerCommand (DrivetrainSubsystem drivetrainSubsystem) {
        TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared).setKinematics(DriveConstants.kDriveKinematics);

        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(new Translation2d(-2, 0)),
            new Pose2d(-4.89, 0, new Rotation2d(0)),
            config
        ); 

        var thetaController = new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, AutoConstants.kDThetaController, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
            trajectory,
            drivetrainSubsystem::getPose, // Functional interface to feed supplier
            DriveConstants.kDriveKinematics,

            // Position controllers
            new PIDController(AutoConstants.kPXController, 0, AutoConstants.kDXController),
            new PIDController(AutoConstants.kPYController, 0, AutoConstants.kDYController),        
            thetaController,
            drivetrainSubsystem::setModuleStates,
            drivetrainSubsystem);

        // Run the "Glass" program and then choose NetworkTables -> SmartDashboard -> Field2d to view the Field.
        // The field image for 2023 is in utils folder
        // Constants.DriveConstants.field.getObject("traj").setTrajectory(trajectory);
        
        swerveControllerCommand.addRequirements(drivetrainSubsystem);
        drivetrainSubsystem.resetOdometry(trajectory.getInitialPose());

        return swerveControllerCommand;
    }
}
