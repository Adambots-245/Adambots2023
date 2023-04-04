// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.commands.autonCommands;

import com.adambots.RobotMap;
import com.adambots.commands.ArmCommands;
import com.adambots.commands.GrabCommand;
import com.adambots.commands.UngrabCommand;
import com.adambots.subsystems.DrivetrainSubsystem;
import com.adambots.subsystems.GrabSubsystem;
import com.adambots.subsystems.GrabbyLifterSubsystem;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

/** Main class for all Auton Commands.
 * Uses new WPILib approach to chain commands.
 */
public class AutonCommands {
    private GrabSubsystem grabSubsystem;
    private GrabbyLifterSubsystem grabbyLifterSubsystem;
    private DrivetrainSubsystem drivetrainSubsystem;
    private ArmCommands armCommands;

    public AutonCommands(GrabSubsystem grabSubsystem, GrabbyLifterSubsystem grabbyLifterSubsystem,
            DrivetrainSubsystem drivetrainSubsystem, ArmCommands armCommands) {
        this.grabSubsystem = grabSubsystem;
        this.grabbyLifterSubsystem = grabbyLifterSubsystem;
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.armCommands = armCommands;
    }

    public Command testWaypoint() {
        Pose2d waypoint1 = getPose(3, 0, 0);

        return Commands.sequence(
            resetGyroCommand(),
            resetOdometryCommand(getPose(0, 0, 0)),
            new DriveToWaypointCommand(drivetrainSubsystem, RobotMap.GyroSensor, waypoint1, 0.5)
        );
    }

    public Command humanStationConePickup() {
        return Commands.sequence(
            new DriveTimeCommand(drivetrainSubsystem, 0.2, 0, 0, true, 0.3),
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
            new DriveTimeCommand(drivetrainSubsystem, 0.2, 0, 0, true, 0.3),
            armCommands.humanStationCubeCommand(),
            new DriveTimeCommand(drivetrainSubsystem, -0.6, 0, 0, true, 0.325),
            new WaitCommand(0.5),
            armCommands.grabCommand(),
            new WaitCommand(0.7),
            armCommands.homeCommand()
        );
    }

    public Command pickupGamePiece(String turnDirection) {
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
            armCommands.highConeCommand(),
            new WaitCommand(1.2),
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

    public Command scorePickupTop() {
        Pose2d waypoint1 = getPose(3.705, -0.397, 160);
        Pose2d waypoint2 = getPose(0, -0.591, 0);

        return Commands.sequence(
            autoInitAndScoreCone(),
            armCommands.homeCommand(),
            new DriveToWaypointCommand(drivetrainSubsystem, RobotMap.GyroSensor, waypoint1, 0.5),
            pickupGamePiece("right"),
            Commands.parallel(
                new DriveToWaypointCommand(drivetrainSubsystem, RobotMap.GyroSensor, waypoint2, 0),
                new WaitCommand(3).andThen(armCommands.highCubeCommand())),
            new DriveTimeCommand(drivetrainSubsystem, 0.5, 0, 0, false, 0.35),
            new UngrabCommand(grabSubsystem),
            new WaitCommand(0.35),
            armCommands.homeCommand()
        );
    }

    public Command scorePickupBottom() {
        Pose2d waypoint1 = getPose(3.705, 0.408, -160);
        Pose2d waypoint2 = getPose(0, 0.558, 0);

        return Commands.sequence(
            autoInitAndScoreCone(),
            armCommands.homeCommand(),
            new DriveToWaypointCommand(drivetrainSubsystem, RobotMap.GyroSensor, waypoint1, 0.5),
            pickupGamePiece("left"),
            Commands.parallel(
                new DriveToWaypointCommand(drivetrainSubsystem, RobotMap.GyroSensor, waypoint2, 0),
                new WaitCommand(3).andThen(armCommands.highCubeCommand())),
            new DriveTimeCommand(drivetrainSubsystem, 0.5, 0, 0, false, 0.35),
            new UngrabCommand(grabSubsystem),
            new WaitCommand(0.35),
            armCommands.homeCommand()
        );
    }

    public Pose2d getPose(double x, double y, double rotDegrees) {
        String allianceColor = DriverStation.getAlliance().name();
        if (allianceColor.toLowerCase() == "red") {
            y *= -1;
            rotDegrees *= -1;
        } else if (allianceColor.toLowerCase() != "blue") {
            System.err.println("Alliance Color not selected (AutonCommands.getPose)");
        }

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
