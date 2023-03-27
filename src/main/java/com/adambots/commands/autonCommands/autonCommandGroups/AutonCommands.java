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

    private final String autoInitAndScoreCubePath = "TopCubeCube1.wpilib.json";

    public Command humanStationPickup() {
        return new DriveTimeCommand(drivetrainSubsystem, 0.1, 0, 0, true, 0.1)
        .andThen(armCommands.humanStationCommand())
        .andThen(new DriveTimeCommand(drivetrainSubsystem, -0.6, 0, 0, true, 0.3))
        .andThen(new WaitCommand(0.5))
        .andThen(armCommands.grabCommand())
        .andThen(new WaitCommand(0.4))
        .andThen(armCommands.homeCommand());
    }

    public Command autoInitAndScoreCube() {
        String trajectoryFileName = autoInitAndScoreCubePath;

        Trajectory trajectory = getTrajectory(trajectoryFileName);
        return autoInitAndScoreCube(trajectory);
    }

    public Command autoInitAndScoreCube(Trajectory trajectory) {

        return resetGyroCommand().andThen(
                initializeFieldTrajectoryCommand(trajectory),
                resetOdometryCommand(trajectory),
                armCommands.highCubeCommand(),
                // Commands.parallel(new ArmLifterChangeStateCommand(grabbyLifterSubsystem, GrabbyConstants.highCubeState),
                        // new FirstExtenderChangeStateCommand(firstExtenderSubsystem, GrabbyConstants.highCubeState),
                        // new SecondExtenderChangeStateCommand(secondExtenderSubsystem, GrabbyConstants.highCubeState)),
                new WaitCommand(1.2),
                new UngrabCommand(grabSubsystem),
                new WaitCommand(0.2)
        );
    }

    public Command autoInitAndScoreCone(Trajectory trajectory){
        
        return resetGyroCommand().andThen(
            initializeFieldTrajectoryCommand(trajectory),
            resetOdometryCommand(trajectory),
            armCommands.highConeCommand(),
            // Commands.parallel(new ArmLifterChangeStateCommand(grabbyLifterSubsystem, GrabbyConstants.highConeState), 
                            //   new FirstExtenderChangeStateCommand(firstExtenderSubsystem, GrabbyConstants.highConeState), 
                            //   new SecondExtenderChangeStateCommand(secondExtenderSubsystem, GrabbyConstants.highConeState)),
            new WaitCommand(1.2),
            new UngrabCommand(grabSubsystem),
            new WaitCommand(0.2)
        );
    }

    public SequentialCommandGroup noTrajInitAndScore() {
        return new SequentialCommandGroup(
                resetGyroCommand(),
                // armCommands.highCubeCommand(),
                // Commands.parallel(new ArmLifterChangeStateCommand(grabbyLifterSubsystem, GrabbyConstants.highCubeState),
                        // new FirstExtenderChangeStateCommand(firstExtenderSubsystem, GrabbyConstants.highCubeState),
                        // new SecondExtenderChangeStateCommand(secondExtenderSubsystem, GrabbyConstants.highCubeState)),
                new WaitCommand(1.4),
                new UngrabCommand(grabSubsystem),
                new WaitCommand(0.3)
                );
    }

    // private final String midCubeChargePath = "Charge.wpilib.json";

    public Command midCubeCharge() {
        // Trajectory trajectory = getTrajectory(midCubeChargePath);

        return Commands.deadline(new WaitCommand(14.6), 
                    // autoInitAndScoreCube(trajectory).andThen(
                    Commands.sequence(noTrajInitAndScore(),
                    Commands.parallel(
                            new FirstExtenderChangeStateCommand(firstExtenderSubsystem, GrabbyConstants.balancingState),
                            new SecondExtenderChangeStateCommand(secondExtenderSubsystem, GrabbyConstants.balancingState)),
                    // new InstantCommand(() -> drivetrainSubsystem.drive(-0.1, 0, 0, false)),
                    new WaitCommand(1),
                    new TraversePlatform(drivetrainSubsystem, RobotMap.GyroSensor),
                    // Functions.CreateSwerveControllerCommand(drivetrainSubsystem, traj1),
                    new AutoBalanceCommand(drivetrainSubsystem, RobotMap.GyroSensor, grabbyLifterSubsystem),
                    new HockeyStopCommand(drivetrainSubsystem))
                ).andThen(new HockeyStopCommand(drivetrainSubsystem));
    }

    private final String topCubePath1 = "TopCubeCube1.wpilib.json";
    private final String topCubePath2 = "TopCubeCube2.wpilib.json"; 
    private final String topCubeScorePath2 = "TopCubeCubeScore2.wpilib.json"; 

    public Command scorePickupTop() {
        Trajectory trajectory1 = getTrajectory(topCubePath1);
        Trajectory trajectory2 = getTrajectory(topCubePath2);
        Trajectory trajectory3 = getTrajectory(topCubeScorePath2);

        // return autoInitAndScoreCube(trajectory1).andThen(
        return Commands.sequence(autoInitAndScoreCone(trajectory1),
                Commands.parallel(
                        new FirstExtenderChangeStateCommand(firstExtenderSubsystem, GrabbyConstants.groundState),
                        new SecondExtenderChangeStateCommand(secondExtenderSubsystem, GrabbyConstants.groundState)),
                new DriveTimeCommand(drivetrainSubsystem, 0.7, -0.3, 0, true, 1),
                Commands.parallel(driveTrajectory(drivetrainSubsystem, trajectory2), 
                                  new WaitCommand(1).andThen(new ArmLifterChangeStateCommand(grabbyLifterSubsystem, GrabbyConstants.groundState))),
                stopDriving(),
                new AutonPickupCommand(drivetrainSubsystem, grabSubsystem, 0.75),
                new WaitCommand(1),
                armCommands.homeCommand(),
                // Commands.parallel(
                //     new ArmLifterChangeStateCommand(grabbyLifterSubsystem, GrabbyConstants.initState), 
                //     new FirstExtenderChangeStateCommand(firstExtenderSubsystem, GrabbyConstants.initState), 
                //     new SecondExtenderChangeStateCommand(secondExtenderSubsystem, GrabbyConstants.initState)),
                Commands.parallel(driveTrajectory(drivetrainSubsystem, trajectory3), 
                    new WaitCommand(3).andThen(
                        armCommands.highCubeCommand()
                        // Commands.parallel(
                        //     new ArmLifterChangeStateCommand(grabbyLifterSubsystem, GrabbyConstants.highCubeState), 
                        //     new FirstExtenderChangeStateCommand(firstExtenderSubsystem, GrabbyConstants.highCubeState), 
                        //     new SecondExtenderChangeStateCommand(secondExtenderSubsystem, GrabbyConstants.highCubeState)
                            )),
    
                new DriveTimeCommand(drivetrainSubsystem, 0.5, 0, 0, false, 0.5),
                stopDriving(),
                new UngrabCommand(grabSubsystem),
                new WaitCommand(0.3),
                armCommands.homeCommand()
                // Commands.parallel(
                //     new ArmLifterChangeStateCommand(grabbyLifterSubsystem, GrabbyConstants.initState), 
                //     new FirstExtenderChangeStateCommand(firstExtenderSubsystem, GrabbyConstants.initState), 
                //     new SecondExtenderChangeStateCommand(secondExtenderSubsystem, GrabbyConstants.initState))
                );
    }

    private final String bottomCubePath1 = "BottomCubeCube1.wpilib.json";
    private final String bottomCubePath2 = "BottomCubeCubeScore2.wpilib.json";

    public Command scorePickupBottom() {
        Trajectory trajectory1 = getTrajectory(bottomCubePath1);
        Trajectory trajectory2 = getTrajectory(bottomCubePath2);

        return Commands.sequence(autoInitAndScoreCube(trajectory1),
                armCommands.homeCommand(),
                // Commands.parallel(
                        // new FirstExtenderChangeStateCommand(firstExtenderSubsystem, GrabbyConstants.initState),
                        // new SecondExtenderChangeStateCommand(secondExtenderSubsystem, GrabbyConstants.initState),
                        // new ArmLifterChangeStateCommand(grabbyLifterSubsystem, GrabbyConstants.initState)),
                new WaitCommand(1.5),
                driveTrajectory(drivetrainSubsystem, trajectory1),
                stopDriving(),
                armCommands.groundCommand(),
                // Commands.parallel(
                //         new FirstExtenderChangeStateCommand(firstExtenderSubsystem, GrabbyConstants.groundState),
                //         new SecondExtenderChangeStateCommand(secondExtenderSubsystem, GrabbyConstants.groundState),
                //         new ArmLifterChangeStateCommand(grabbyLifterSubsystem, GrabbyConstants.groundState)),
                new WaitCommand(1.5),
                new AutonPickupCommand(drivetrainSubsystem, grabSubsystem, 2.2),
                new WaitCommand(0.75),
                armCommands.homeCommand(),
                // Commands.parallel(
                //         new ArmLifterChangeStateCommand(grabbyLifterSubsystem, GrabbyConstants.initState),
                //         new FirstExtenderChangeStateCommand(firstExtenderSubsystem, GrabbyConstants.initState),
                //         new SecondExtenderChangeStateCommand(secondExtenderSubsystem, GrabbyConstants.initState)),
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

    public SwerveControllerCommand driveTrajectory(DrivetrainSubsystem drivetrainSubsystem,
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

    public Command stopDriving(){
        return new InstantCommand(() -> drivetrainSubsystem.stop());
    }
}
