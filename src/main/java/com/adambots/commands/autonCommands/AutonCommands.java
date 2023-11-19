// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.commands.autonCommands;

import java.io.IOException;
import java.nio.file.Path;

import com.adambots.Constants.AutoConstants;
import com.adambots.Constants.DriveConstants;
import com.adambots.RobotMap;
import com.adambots.commands.ArmCommands;
import com.adambots.commands.GrabCommand;
import com.adambots.commands.UngrabCommand;
import com.adambots.subsystems.DrivetrainSubsystem;
import com.adambots.subsystems.FirstExtenderSubsystem;
import com.adambots.subsystems.GrabSubsystem;
import com.adambots.subsystems.GrabbyLifterSubsystem;
import com.pathplanner.lib.auto.*;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

/** Main class for all Auton Commands.
 * Uses new WPILib approach to chain commands.
 */
public class AutonCommands {
    private GrabSubsystem grabSubsystem;
    private FirstExtenderSubsystem firstExtenderSubsystem;

    private GrabbyLifterSubsystem grabbyLifterSubsystem;
    private DrivetrainSubsystem drivetrainSubsystem;
    private ArmCommands armCommands;

    public static enum Direction{
        LEFT,
        RIGHT
      }

    public AutonCommands(GrabSubsystem grabSubsystem, GrabbyLifterSubsystem grabbyLifterSubsystem,
            DrivetrainSubsystem drivetrainSubsystem, ArmCommands armCommands, FirstExtenderSubsystem firstExtenderSubsystem) {
        this.firstExtenderSubsystem = firstExtenderSubsystem;
        this.grabSubsystem = grabSubsystem;
        this.grabbyLifterSubsystem = grabbyLifterSubsystem;
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.armCommands = armCommands;

        AutoBuilder.configureHolonomic(
            drivetrainSubsystem::getPose, // Robot pose supplier
            drivetrainSubsystem::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
            drivetrainSubsystem::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            drivetrainSubsystem::setChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig(
                new PIDConstants(AutoConstants.kPXController, 0, AutoConstants.kDXController), 
                new PIDConstants(AutoConstants.kPThetaController, 0, AutoConstants.kDThetaController),
                4.5, // Max module speed, in m/s
                0.43, // Drive base radius in meters. Distance from robot center to furthest module.
                new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
        drivetrainSubsystem // Reference to this subsystem to set requirements
        );
    }

    

    public Command testWaypoint() {
        // Pose2d waypoint1 = getPose(-2, -1, 90);

        return Commands.sequence(
            // resetGyroCommand(),
            // resetOdometryCommand(getPose(0.1, 0, 0))
            // new DriveToWaypointCommand(drivetrainSubsystem, RobotMap.GyroSensor, waypoint1, 0)
        );
    }

    public Command groundConePickup() {
        return Commands.sequence(
            new DriveTimeCommand(drivetrainSubsystem, 0.17, 0, 0, false, 0.35),
            new DriveTimeCommand(drivetrainSubsystem, -0.2, 0, 0, false, 0.225),
            armCommands.grabCommand()
        );
    }

    public Command humanPlayerClamp() {
        return Commands.sequence(
            armCommands.grabCommand(),
            new WaitCommand(0.4),
            armCommands.humanStationClamp()
        );
    }

    public Command humanStationConePickup() {
        return Commands.sequence(
            // new DriveTimeCommand(drivetrainSubsystem, 0.325, 0, 0, true, 0.65),
            armCommands.humanStationConeCommand()
            // new DriveTimeCommand(drivetrainSubsystem, -0.6, 0, 0, true, 0.325),
            // new WaitCommand(0.5),
            // armCommands.grabCommand(),
            // new WaitCommand(0.4),
            // armCommands.homeCommand()
        );
    }

    public Command humanStationBackConePickup() {
        return Commands.sequence(
            new DriveTimeCommand(drivetrainSubsystem, 0.325, 0, 0, true, 0.65),
            armCommands.humanStationConeCommand(),
            new DriveTimeCommand(drivetrainSubsystem, -0.6, 0, 0, true, 0.325),
            new WaitCommand(0.5),
            armCommands.grabCommand(),
            new WaitCommand(0.4),
            armCommands.homeCommand()
        );
    }

    public Command humanStationCubePickup() {
        return Commands.sequence(
            // new DriveTimeCommand(drivetrainSubsystem, 0.2, 0, 0, true, 0.3),
            // Commands.deadline(new WaitCommand(1.5), new DriveTillBumped(drivetrainSubsystem)),
            armCommands.humanStationCubeCommand()
            // new DriveTimeCommand(drivetrainSubsystem, -0.6, 0, 0, true, 0.325),
            // new WaitCommand(0.5),
            // armCommands.grabCommand(),
            // new WaitCommand(0.7),
            // armCommands.homeCommand()
        );
    }

    public Command pickupGamePiece(Direction turnDirection) {
        return Commands.sequence(
            new TurnToGamePieceCommand(drivetrainSubsystem, RobotMap.lidar, turnDirection),
            armCommands.groundCommand(),
            new DriveToGamePieceCommand(drivetrainSubsystem, RobotMap.lidar, grabbyLifterSubsystem),
            new GrabCommand(grabSubsystem),
            new WaitCommand(0.2),
            armCommands.homeCommand()
        );
    }

    public Command autoInitAndScoreCube() {
        return Commands.sequence(
            resetGyroCommand(),
            resetOdometryCommand(getPose(0, 0, 0)),
            armCommands.highCubeCommand(),
            new WaitCommand(1.2),
            new UngrabCommand(grabSubsystem),
            new WaitCommand(0.3)
        );
    }

    public Command autoInitAndScoreCone(){
        return Commands.sequence(
            resetGyroCommand(),
            resetOdometryCommand(getPose(0, 0, 0)),
            armCommands.autonHighConeCommand(),
            new WaitCommand(0.9),
            new UngrabCommand(grabSubsystem),
            new WaitCommand(0.3)
        );
    }

    public Command midCubeCharge() {
        return Commands.deadline(
            new WaitCommand(14.7), 
            Commands.sequence(
                autoInitAndScoreCube(),
                armCommands.homeCommand(),
                new TraversePlatform(drivetrainSubsystem, RobotMap.GyroSensor),
                // Commands.deadline(
                //     new WaitCommand(4),
                //     pickupGamePiece("cube")),
                new AutoBalanceCommand(drivetrainSubsystem, RobotMap.GyroSensor, grabbyLifterSubsystem),
                new HockeyStopCommand(drivetrainSubsystem)
            )
        ).andThen(new HockeyStopCommand(drivetrainSubsystem));
    }

    public Command scorePickupTopBlue() {
        Pose2d waypoint1 = getPose(-4.75, 0.5, 165);
        Pose2d waypoint2 = getPose(-0.1, 0.9, -11);
        Pose2d waypoint3 = getPose(-6, 1, -4.5);

        return Commands.sequence(
            autoInitAndScoreCone(),
            armCommands.homeCommand(),
            new DriveToWaypointCommand(drivetrainSubsystem, RobotMap.GyroSensor, waypoint1, 0.25),
            pickupGamePiece(Direction.RIGHT),
            new DriveTimeCommand(drivetrainSubsystem, -0.4, 0.35, 0, true, 0.35),
            Commands.parallel(
                new DriveToWaypointCommand(drivetrainSubsystem, RobotMap.GyroSensor, waypoint2, 0),
                new WaitCommand(2.25).andThen(armCommands.highCubeCommand())),
            new DriveTimeCommand(drivetrainSubsystem, -0.3, 0, 0, true, 0.6),
            new WaitCommand(1),
            new UngrabCommand(grabSubsystem),
            new WaitCommand(0.3),
            armCommands.homeCommand()
            // new DriveToWaypointCommand(drivetrainSubsystem, RobotMap.GyroSensor, waypoint3, 0)
        );
    }

    public Command scorePickupTopRed() {
        Pose2d waypoint1 = getPose(-4.75, -0.5, -165);
        Pose2d waypoint2 = getPose(-0.1, -0.9, 11);
        Pose2d waypoint3 = getPose(-6, -1, 4.5);

        return Commands.sequence(
            autoInitAndScoreCone(),
            armCommands.homeCommand(),
            new DriveToWaypointCommand(drivetrainSubsystem, RobotMap.GyroSensor, waypoint1, 0.25),
            pickupGamePiece(Direction.LEFT),
            new DriveTimeCommand(drivetrainSubsystem, -0.4, -0.35, 0, true, 0.35),
            Commands.parallel(
                new DriveToWaypointCommand(drivetrainSubsystem, RobotMap.GyroSensor, waypoint2, 0),
                new WaitCommand(2.25).andThen(armCommands.highCubeCommand())),
            new DriveTimeCommand(drivetrainSubsystem, -0.3, -0, 0, true, 0.6),
            new WaitCommand(1),
            new UngrabCommand(grabSubsystem),
            new WaitCommand(0.3),
            armCommands.homeCommand()
            // new DriveToWaypointCommand(drivetrainSubsystem, RobotMap.GyroSensor, waypoint3, 0)
        );
    }

    // private final String bottomCubePath1 = "Test2.wpilib.json";

    public Command scorePickupBottomBlue() {

        // ArrayList<PathPlannerTrajectory> pathGroup = (ArrayList<PathPlannerTrajectory>) PathPlanner.loadPathGroup("Test3",
        //     new PathConstraints(1, 1));

        return Commands.sequence(
            resetGyroCommand()
            // resetOdometryCommand(pathGroup.get(0).getInitialPose()),
            // autoBuilder.fullAuto(pathGroup)
            // autoInitAndScoreCube(trajectory1),
            // armCommands.homeCommand(),
            // new WaitCommand(1.5),
            // driveTrajectory(drivetrainSubsystem, trajectory1),
            // stopDriving()
            // armCommands.groundCommand(),
            // new WaitCommand(1.5),
            // new AutonPickupCommand(drivetrainSubsystem, grabSubsystem, 2.2),
            // new WaitCommand(0.75),
            // armCommands.homeCommand(),
            // driveTrajectory(drivetrainSubsystem, trajectory2)
        );
    }

    public Command scorePickupBottomRed() {
        Pose2d waypoint1 = getPose(-5, 0.5, -160); //-155
        Pose2d waypoint2 = getPose(-0.15, 0.15, -10);

        return Commands.deadline(new WaitCommand(14.8), Commands.sequence(
            Commands.parallel(
                new DriveTimeCommand(drivetrainSubsystem, 0.2, 0, 0, false, 0.5),
                autoInitAndScoreCone()),
            armCommands.homeCommand(),
            new DriveTimeCommand(drivetrainSubsystem, 0.3, -0.1, 0, true, 0.2),
            new DriveTimeCommand(drivetrainSubsystem, 0.8, -0.15, 0, true, 0.95),
            new DriveTimeCommand(drivetrainSubsystem, 0.45, -0, 0, true, 1.4),
            new DriveToWaypointCommand(drivetrainSubsystem, RobotMap.GyroSensor, waypoint1, 0),
            pickupGamePiece(Direction.LEFT),
            Commands.parallel(
                new DriveToWaypointCommand(drivetrainSubsystem, RobotMap.GyroSensor, waypoint2, 0),
                new WaitCommand(2.5).andThen(armCommands.highCubeCommand())
            ),
            new DriveTimeCommand(drivetrainSubsystem, -0.4, -0, 0, true, 0.8),
            new WaitCommand(0.2),
            new UngrabCommand(grabSubsystem),
            new WaitCommand(0.3),
            armCommands.homeCommand()
        )).andThen(new UngrabCommand(grabSubsystem));
    }

    public Trajectory getTrajectory(String trajectoryName) {
        Trajectory trajectory = new Trajectory();
        String allianceColor = DriverStation.getAlliance().toString();

        // // If the trajectoryname already has Blue or Red in it, don't mess with it //Previously were getting errors maybe relating to this, commenting it out for now
        // if (!(trajectoryName.startsWith("Blue") || trajectoryName.startsWith("Red"))) {

        // trajectoryName = allianceColor + trajectoryName;
        // }

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
            new PIDController(AutoConstants.kPXController, 0, AutoConstants.kDXController),
            thetaController,
            // () -> new Rotation2d(Math.toRadians(0)),
            drivetrainSubsystem::setModuleStates,
            drivetrainSubsystem
        );

        // Run the "Glass" program and then choose NetworkTables -> SmartDashboard ->
        // Field2d to view the Field.
        // The field image for 2023 is in utils folder
        DriveConstants.field.getObject("traj").setTrajectory(trajectory);

        return swerveControllerCommand;
    }

    public Pose2d getPose(double x, double y, double rotDegrees) {
        // Alliance allianceColor = DriverStation.getAlliance();
        // if (allianceColor == Alliance.Red) {
        //     y *= -1;
        //     rotDegrees *= -1;
        // }

        return new Pose2d(new Translation2d(x, y), new Rotation2d(Math.toRadians(rotDegrees)));
    }

    public static class Waypoint{
        private Pose2d pose;
        private double switchAtTimeSec;

        public Waypoint(Pose2d pose, double switchAtTimeSec){
            this.pose = pose;
            this.switchAtTimeSec = switchAtTimeSec;
        }

        public Pose2d pose(){
            return pose;
        }

        public double switchAtTime(){
            return switchAtTimeSec;
        }
    }

    public Command driveTillBumpedCommand() {
        return new DriveTillBumped(drivetrainSubsystem);
    }

    public Command resetOdometryCommand(Pose2d pose) {
        return new InstantCommand(() -> drivetrainSubsystem.resetOdometry(pose));
    }

    public Command resetGyroCommand() {
        return new InstantCommand(() -> RobotMap.GyroSensor.reset());
    }

    public Command stopDriving(){
        return new InstantCommand(() -> drivetrainSubsystem.stop());
    }
}
