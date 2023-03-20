// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.commands.autonCommands.autonCommandGroups;

import java.io.IOException;
import java.nio.file.Path;

import com.adambots.Constants.*;
import com.adambots.RobotMap;
import com.adambots.commands.*;
import com.adambots.commands.autonCommands.*;
import com.adambots.subsystems.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.*;

/** Main class for all Auton Commands.
 * Uses new WPILib approach to chain commands.
 */
public class AutonCommands {
    private GrabSubsystem grabSubsystem;
    private GrabbyLifterSubsystem grabbyLifterSubsystem;
    private FirstExtenderSubsystem firstExtenderSubsystem;
    private SecondExtenderSubsystem secondExtenderSubsystem;
    private DrivetrainSubsystem drivetrainSubsystem;
    private CANdleSubsystem ledSubsystem;

    public AutonCommands(GrabSubsystem grabSubsystem, GrabbyLifterSubsystem grabbyLifterSubsystem,
            FirstExtenderSubsystem firstExtenderSubsystem, SecondExtenderSubsystem secondExtenderSubsystem,
            DrivetrainSubsystem drivetrainSubsystem, CANdleSubsystem ledSubsystem) {
        this.grabSubsystem = grabSubsystem;
        this.grabbyLifterSubsystem = grabbyLifterSubsystem;
        this.firstExtenderSubsystem = firstExtenderSubsystem;
        this.secondExtenderSubsystem = secondExtenderSubsystem;
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.ledSubsystem = ledSubsystem;
    }

    private final String autoInitAndScorePath = "TopCubeCube1.wpilib.json";

    public Command autoInitAndScoreCube() {
        String trajectoryFileName = autoInitAndScorePath;

        Trajectory trajectory = getTrajectory(trajectoryFileName);
        return autoInitAndScoreCube(trajectory);
    }

    public Command autoInitAndScoreCube(Trajectory trajectory) {

        return resetGyroCommand().andThen(
                initializeFieldTrajectoryCommand(trajectory),
                resetOdometryCommand(trajectory),
                Commands.parallel(new ArmLifterChangeStateCommand(grabbyLifterSubsystem, GrabbyConstants.highCubeState),
                        new FirstExtenderChangeStateCommand(firstExtenderSubsystem, GrabbyConstants.highCubeState),
                        new SecondExtenderChangeStateCommand(secondExtenderSubsystem, GrabbyConstants.highCubeState)),
                new WaitCommand(1.7),
                new UngrabCommand(grabSubsystem),
                new WaitCommand(0.3)
                );
    }

    private final String midCubeChargePath = "Charge.wpilib.json";

    public Command midCubeCharge() {
        Trajectory trajectory = getTrajectory(midCubeChargePath);

        return autoInitAndScoreCube(trajectory).andThen(
                Commands.parallel(
                        new FirstExtenderChangeStateCommand(firstExtenderSubsystem, GrabbyConstants.balancingState),
                        new SecondExtenderChangeStateCommand(secondExtenderSubsystem, GrabbyConstants.balancingState)),
                // new InstantCommand(() -> drivetrainSubsystem.drive(-0.1, 0, 0, false)),
                new WaitCommand(1),
                // Functions.CreateSwerveControllerCommand(drivetrainSubsystem, traj1),
                new TestAutoBalanceCommand(drivetrainSubsystem, RobotMap.GyroSensor, grabbyLifterSubsystem),
                // new AutoBalanceCommand(drivetrainSubsystem, gyro)
                new HockeyStopCommand(drivetrainSubsystem)
                );
    }

    private final String topCubePath1 = "TopCubeCube1.wpilib.json";
    private final String topCubePath2 = "Testing.wpilib.json"; // TODO: Change name so that there's one for Blue and Red

    public Command scorePickupTop() {
        Trajectory trajectory1 = getTrajectory(topCubePath1);
        Trajectory trajectory2 = getTrajectory(topCubePath2);

        return autoInitAndScoreCube(trajectory1).andThen(
                Commands.parallel(
                        new FirstExtenderChangeStateCommand(firstExtenderSubsystem, GrabbyConstants.groundState),
                        new SecondExtenderChangeStateCommand(secondExtenderSubsystem, GrabbyConstants.groundState)),
                // Commands.parallel(Functions.CreateSwerveControllerCommand(drivetrainSubsystem,
                // traj1).andThen(new InstantCommand(() -> drivetrainSubsystem.stop())), new
                // WaitCommand(1.75).andThen(new
                // ArmLifterChangeStateCommand(grabbyLifterSubsystem,
                // GrabbyConstants.groundState))),
                CreateSwerveControllerCommand(drivetrainSubsystem, trajectory1),
                new InstantCommand(() -> drivetrainSubsystem.stop()),
                new ArmLifterChangeStateCommand(grabbyLifterSubsystem, GrabbyConstants.groundState),
                new WaitCommand(1.5),
                new AutonPickupCommand(drivetrainSubsystem, grabSubsystem, 1),
                new WaitCommand(1),
                Commands.parallel(new ArmLifterChangeStateCommand(grabbyLifterSubsystem, GrabbyConstants.initState),
                        new FirstExtenderChangeStateCommand(firstExtenderSubsystem, GrabbyConstants.initState),
                        new SecondExtenderChangeStateCommand(secondExtenderSubsystem, GrabbyConstants.initState)),
                CreateSwerveControllerCommand(drivetrainSubsystem, trajectory2)
                );
    }

    private final String bottomCubePath1 = "BottomCubeCube1.wpilib.json";
    private final String bottomCubePath2 = "BottomCubeCubeScore2.wpilib.json";

    public Command scorePickupBottom() {
        Trajectory trajectory1 = getTrajectory(bottomCubePath1);
        Trajectory trajectory2 = getTrajectory(bottomCubePath2);

        return autoInitAndScoreCube(trajectory1).andThen(
                Commands.parallel(
                        new FirstExtenderChangeStateCommand(firstExtenderSubsystem, GrabbyConstants.initState),
                        new SecondExtenderChangeStateCommand(secondExtenderSubsystem, GrabbyConstants.initState),
                        new ArmLifterChangeStateCommand(grabbyLifterSubsystem, GrabbyConstants.initState)),
                new WaitCommand(3),
                CreateSwerveControllerCommand(drivetrainSubsystem, trajectory1),
                new InstantCommand(() -> drivetrainSubsystem.stop()),
                Commands.parallel(
                        new FirstExtenderChangeStateCommand(firstExtenderSubsystem, GrabbyConstants.groundState),
                        new SecondExtenderChangeStateCommand(secondExtenderSubsystem, GrabbyConstants.groundState),
                        new ArmLifterChangeStateCommand(grabbyLifterSubsystem, GrabbyConstants.groundState)),
                new WaitCommand(3),
                new AutonPickupCommand(drivetrainSubsystem, grabSubsystem, 2.5),
                new WaitCommand(0.75),
                Commands.parallel(
                        new ArmLifterChangeStateCommand(grabbyLifterSubsystem, GrabbyConstants.initState),
                        new FirstExtenderChangeStateCommand(firstExtenderSubsystem, GrabbyConstants.initState),
                        new SecondExtenderChangeStateCommand(secondExtenderSubsystem, GrabbyConstants.initState)),
                CreateSwerveControllerCommand(drivetrainSubsystem, trajectory2)
                );
    }

    // Common Functions

    public Trajectory getTrajectory(String trajectoryName) {
        Trajectory trajectory = new Trajectory();
        String allianceColor = DriverStation.getAlliance().name();

        // If the trajectoryname already has Blue or Red in it, don't mess with it
        if (!(trajectoryName.startsWith("Blue") || trajectoryName.startsWith("Red"))) {

            trajectoryName = allianceColor + trajectoryName;
        }

        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryName);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + trajectoryName, ex.getStackTrace());
        }

        return trajectory;
    }

    public SwerveControllerCommand CreateSwerveControllerCommand(DrivetrainSubsystem drivetrainSubsystem,
            Trajectory trajectory) {
        var thetaController = new ProfiledPIDController(
                AutoConstants.kPThetaController, 0, AutoConstants.kDThetaController,
                AutoConstants.kThetaControllerConstraints);
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

        // Run the "Glass" program and then choose NetworkTables -> SmartDashboard ->
        // Field2d to view the Field.
        // The field image for 2023 is in utils folder
        DriveConstants.field.getObject("traj").setTrajectory(trajectory);

        return swerveControllerCommand;
    }

    private Command initializeFieldTrajectoryCommand(Trajectory trajectory) {
        return new InstantCommand(() -> DriveConstants.field.getObject("traj").setTrajectory(trajectory));
    }

    private Command resetOdometryCommand(Trajectory trajectory) {
        return new InstantCommand(() -> drivetrainSubsystem.resetOdometry(trajectory.getInitialPose()));
    }

    private Command resetGyroCommand() {
        return new InstantCommand(() -> RobotMap.GyroSensor.reset());
    }
}
