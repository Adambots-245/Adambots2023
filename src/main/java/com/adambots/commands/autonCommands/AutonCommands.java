// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.commands.autonCommands;

import java.io.IOException;
import java.nio.file.Path;

import com.adambots.Constants.*;
import com.adambots.RobotMap;
import com.adambots.commands.*;
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
    private ArmCommands armCommands;

    public AutonCommands(GrabSubsystem grabSubsystem, GrabbyLifterSubsystem grabbyLifterSubsystem,
            FirstExtenderSubsystem firstExtenderSubsystem, SecondExtenderSubsystem secondExtenderSubsystem,
            DrivetrainSubsystem drivetrainSubsystem, ArmCommands armCommands) {
        this.grabSubsystem = grabSubsystem;
        this.grabbyLifterSubsystem = grabbyLifterSubsystem;
        this.firstExtenderSubsystem = firstExtenderSubsystem;
        this.secondExtenderSubsystem = secondExtenderSubsystem;
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.armCommands = armCommands;
    }

    public Command humanStationPickup() {
        return Commands.sequence(
            new DriveTimeCommand(drivetrainSubsystem, 0.1, 0, 0, true, 0.1),
            // armCommands.humanStationCommand(), //NEED TO MAKE SURE THESE ARE FIXED FOR TELEOP
            new ParallelCommandGroup(
                new ArmLifterChangeStateCommand(grabbyLifterSubsystem, GrabbyConstants.humanStationState),
                new FirstExtenderChangeStateCommand(firstExtenderSubsystem, GrabbyConstants.humanStationState),
                new SecondExtenderChangeStateCommand(secondExtenderSubsystem, GrabbyConstants.humanStationState)),
            new DriveTimeCommand(drivetrainSubsystem, -0.6, 0, 0, true, 0.3),
            new WaitCommand(0.5),
            armCommands.grabCommand(),
            new WaitCommand(0.4),
            new ParallelCommandGroup(
                new FirstExtenderChangeStateCommand(firstExtenderSubsystem, GrabbyConstants.initState),
                new SecondExtenderChangeStateCommand(secondExtenderSubsystem, GrabbyConstants.initState),
                new ArmLifterChangeStateCommand(grabbyLifterSubsystem, GrabbyConstants.initState))
        );
    }

    public Command pickupGamePiece(String pieceType) {
        return Commands.sequence(
            new TurnToObjectCommand(drivetrainSubsystem, pieceType),
            // armCommands.groundCommand(),
            new ParallelCommandGroup(
                new ArmLifterChangeStateCommand(grabbyLifterSubsystem, GrabbyConstants.groundState),
                new FirstExtenderChangeStateCommand(firstExtenderSubsystem, GrabbyConstants.groundState),
                new SecondExtenderChangeStateCommand(secondExtenderSubsystem, GrabbyConstants.groundState)),
            new DriveToDistanceCommand(drivetrainSubsystem, RobotMap.lidar, grabbyLifterSubsystem),
            new GrabCommand(grabSubsystem),
            new WaitCommand(0.2),
            new ParallelCommandGroup(
                new FirstExtenderChangeStateCommand(firstExtenderSubsystem, GrabbyConstants.initState),
                new SecondExtenderChangeStateCommand(secondExtenderSubsystem, GrabbyConstants.initState),
                new ArmLifterChangeStateCommand(grabbyLifterSubsystem, GrabbyConstants.initState))
        );
    }

    public Command autoInitAndScoreCube() {
        return Commands.sequence(
            resetGyroCommand(),
            armCommands.highCubeCommand(),
            new WaitCommand(1.2),
            new UngrabCommand(grabSubsystem),
            new WaitCommand(0.2)
        );
    }

    public Command autoInitAndScoreCube(Trajectory trajectory) {
        return Commands.sequence(
            resetGyroCommand(),
            initializeFieldTrajectoryCommand(trajectory),
            resetOdometryCommand(trajectory),
            armCommands.highCubeCommand(),
            new WaitCommand(1.2),
            new UngrabCommand(grabSubsystem),
            new WaitCommand(0.2)
        );
    }

    public Command autoInitAndScoreCone(){
        return Commands.sequence(
            resetGyroCommand(),
            armCommands.highConeCommand(),
            new WaitCommand(1.2),
            new UngrabCommand(grabSubsystem),
            new WaitCommand(0.2)
        );
    }

    public Command autoInitAndScoreCone(Trajectory trajectory){
        return Commands.sequence(
            resetGyroCommand(),
            initializeFieldTrajectoryCommand(trajectory),
            resetOdometryCommand(trajectory),
            armCommands.highConeCommand(),
            new WaitCommand(1.2),
            new UngrabCommand(grabSubsystem),
            new WaitCommand(0.2)
        );
    }

    public Command midCubeCharge() {
        return Commands.deadline(
            new WaitCommand(14.6), 
            Commands.sequence(
                autoInitAndScoreCube(),
                armCommands.homeCommand(),
                new WaitCommand(1),
                new TraversePlatform(drivetrainSubsystem, RobotMap.GyroSensor),
                new AutoBalanceCommand(drivetrainSubsystem, RobotMap.GyroSensor, grabbyLifterSubsystem),
                new HockeyStopCommand(drivetrainSubsystem)
            )
        ).andThen(new HockeyStopCommand(drivetrainSubsystem));
    }

    private final String topCubePath1 = "TopCubeCube1.wpilib.json";
    private final String topCubePath2 = "TopCubeCube2.wpilib.json"; 
    private final String topCubeScorePath2 = "TopCubeCubeScore2.wpilib.json"; 

    public Command scorePickupTop() {
        Trajectory trajectory1 = getTrajectory(topCubePath1);
        Trajectory trajectory2 = getTrajectory(topCubePath2);
        Trajectory trajectory3 = getTrajectory(topCubeScorePath2);

        return Commands.sequence(
            autoInitAndScoreCone(trajectory1),
            armCommands.homeCommand(),
            new DriveTimeCommand(drivetrainSubsystem, 0.7, -0.3, 0, true, 1),
            Commands.parallel(
                driveTrajectory(drivetrainSubsystem, trajectory2), 
                new WaitCommand(1).andThen(armCommands.groundCommand())),
            stopDriving(),
            new AutonPickupCommand(drivetrainSubsystem, grabSubsystem, 0.75),
            new WaitCommand(1),
            armCommands.homeCommand(),
            Commands.parallel(
                driveTrajectory(drivetrainSubsystem, trajectory3), 
                new WaitCommand(3).andThen(armCommands.highCubeCommand())),
            new DriveTimeCommand(drivetrainSubsystem, 0.5, 0, 0, false, 0.5),
            stopDriving(),
            new UngrabCommand(grabSubsystem),
            new WaitCommand(0.3),
            armCommands.homeCommand()
        );
    }

    private final String bottomCubePath1 = "BottomCubeCube1.wpilib.json";
    private final String bottomCubePath2 = "BottomCubeCubeScore2.wpilib.json";

    public Command scorePickupBottom() {
        Trajectory trajectory1 = getTrajectory(bottomCubePath1);
        Trajectory trajectory2 = getTrajectory(bottomCubePath2);

        return Commands.sequence(
            autoInitAndScoreCube(trajectory1),
            armCommands.homeCommand(),
            new WaitCommand(1.5),
            driveTrajectory(drivetrainSubsystem, trajectory1),
            stopDriving(),
            armCommands.groundCommand(),
            new WaitCommand(1.5),
            new AutonPickupCommand(drivetrainSubsystem, grabSubsystem, 2.2),
            new WaitCommand(0.75),
            armCommands.homeCommand(),
            driveTrajectory(drivetrainSubsystem, trajectory2)
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

    public SwerveControllerCommand driveTrajectory(DrivetrainSubsystem drivetrainSubsystem, Trajectory trajectory) {
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
            drivetrainSubsystem
        );

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

    public Command stopDriving(){
        return new InstantCommand(() -> drivetrainSubsystem.stop());
    }
}
