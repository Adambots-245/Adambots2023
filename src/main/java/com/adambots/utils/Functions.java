package com.adambots.utils;

import java.io.IOException;
import java.nio.file.Path;

import com.adambots.Constants;
import com.adambots.Constants.AutoConstants;
import com.adambots.Constants.DriveConstants;
import com.adambots.subsystems.DrivetrainSubsystem;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class Functions {
    public static SwerveControllerCommand CreateTrajectoryCommand (DrivetrainSubsystem drivetrainSubsystem, String filepath) {
        Trajectory exampleTrajectory = new Trajectory();

        try {
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(filepath);
        exampleTrajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
        DriverStation.reportError("Unable to open trajectory: " + filepath, ex.getStackTrace());
        }

        var thetaController = new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, AutoConstants.kDThetaController, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
            exampleTrajectory,
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
        Constants.DriveConstants.field.getObject("traj").setTrajectory(exampleTrajectory);
        
        return swerveControllerCommand;
    }
}
