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
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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

    public static enum Direction{
        LEFT,
        RIGHT
      }

    public AutonCommands(GrabSubsystem grabSubsystem, GrabbyLifterSubsystem grabbyLifterSubsystem,
            DrivetrainSubsystem drivetrainSubsystem, ArmCommands armCommands) {
        this.grabSubsystem = grabSubsystem;
        this.grabbyLifterSubsystem = grabbyLifterSubsystem;
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.armCommands = armCommands;
    }

    public Command testWaypoint() {
        Pose2d waypoint1 = getPose(-2, -1, 90);

        return Commands.sequence(
            // resetGyroCommand(),
            // resetOdometryCommand(getPose(0.1, 0, 0))
            // new DriveToWaypointCommand(drivetrainSubsystem, RobotMap.GyroSensor, waypoint1, 0)
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

    public Command scorePickupTopBlue() {
        Pose2d waypoint1 = getPose(-4.6, 0.5, 165);
        Pose2d waypoint2 = getPose(-0.4, 0.6, -12);
        Pose2d waypoint3 = getPose(-6, 1, -4.5);

        return Commands.sequence(
            autoInitAndScoreCone(),
            armCommands.homeCommand(),
            new DriveToWaypointCommand(drivetrainSubsystem, RobotMap.GyroSensor, waypoint1, 0.25),
            pickupGamePiece(Direction.RIGHT),
            new DriveTimeCommand(drivetrainSubsystem, 0, 0.6, 0, true, 0.5),
            Commands.parallel(
                new DriveToWaypointCommand(drivetrainSubsystem, RobotMap.GyroSensor, waypoint2, 0),
                new WaitCommand(2.5).andThen(armCommands.highCubeCommand())),
            new DriveTimeCommand(drivetrainSubsystem, 0.55, 0, 0, false, 0.6),
            new WaitCommand(1),
            new UngrabCommand(grabSubsystem),
            new WaitCommand(0.3),
            armCommands.homeCommand()
            // new DriveToWaypointCommand(drivetrainSubsystem, RobotMap.GyroSensor, waypoint3, 0)
            // new DriveTimeCommand(drivetrainSubsystem, -1, 0.125, 0, false, 2.5)
        );
    }

    public Command scorePickupTopRed() {
        Pose2d waypoint1 = getPose(-5, -0.5, -165);
        Pose2d waypoint2 = getPose(-0.4, -0.55, 16);
        Pose2d waypoint3 = getPose(-6, -1, 4.5);

        return Commands.sequence(
            autoInitAndScoreCone(),
            armCommands.homeCommand(),
            new DriveToWaypointCommand(drivetrainSubsystem, RobotMap.GyroSensor, waypoint1, 0.25),
            pickupGamePiece(Direction.LEFT),
            // new DriveTimeCommand(drivetrainSubsystem, 0, -0.6, 0, true, 0.5),
            Commands.parallel(
                new DriveToWaypointCommand(drivetrainSubsystem, RobotMap.GyroSensor, waypoint2, 0),
                new WaitCommand(2.5).andThen(armCommands.highCubeCommand())),
            new DriveTimeCommand(drivetrainSubsystem, 0.55, -0, 0, false, 0.6),
            new WaitCommand(1),
            new UngrabCommand(grabSubsystem),
            new WaitCommand(0.3),
            armCommands.homeCommand()
            // new DriveToWaypointCommand(drivetrainSubsystem, RobotMap.GyroSensor, waypoint3, 0)
            // new DriveTimeCommand(drivetrainSubsystem, -1, 0.125, 0, false, 2.5)
        );
    }

    public Command scorePickupBottomBlue() {
        Pose2d waypoint1 = getPose(-4.75, -0.13, 160);
        Pose2d waypoint2 = getPose(-0.15, -0.2, 10.5);

        return Commands.sequence(
            autoInitAndScoreCone(),
            armCommands.homeCommand(),
            new DriveTimeCommand(drivetrainSubsystem, 0.3, 0.1, 0, true, 0.2),
            new DriveTimeCommand(drivetrainSubsystem, 0.8, 0.2, 0, true, 0.95),
            new DriveTimeCommand(drivetrainSubsystem, 0.45, 0, 0, true, 1.4),
            new DriveToWaypointCommand(drivetrainSubsystem, RobotMap.GyroSensor, waypoint1, 0),
            pickupGamePiece(Direction.RIGHT),
            // new DriveTimeCommand(drivetrainSubsystem, -0.8, -0.1, 0, true, 1.25),
            // new DriveTimeCommand(drivetrainSubsystem, -0.25, 0, 0, true, 2.3),
            Commands.parallel(
                new DriveToWaypointCommand(drivetrainSubsystem, RobotMap.GyroSensor, waypoint2, 0),
                new WaitCommand(2.5).andThen(armCommands.highCubeCommand())
            ),
            new DriveTimeCommand(drivetrainSubsystem, -0.5, 0, 0, true, 1),
            new WaitCommand(0.7),
            // new DriveTimeCommand(drivetrainSubsystem, 0.5, 0, 0, false, 0.35),
            new UngrabCommand(grabSubsystem),
            new WaitCommand(0.3),
            armCommands.homeCommand()
        );
    }

    public Command scorePickupBottomRed() { //TODO: FLIP FOR BLUE
        Pose2d waypoint1 = getPose(-5, 0.35, -150);
        Pose2d waypoint2 = getPose(-0.15, 0.5, -8);

        return Commands.sequence(
            autoInitAndScoreCone(),
            armCommands.homeCommand(),
            new DriveTimeCommand(drivetrainSubsystem, 0.3, -0.1, 0, true, 0.2),
            new DriveTimeCommand(drivetrainSubsystem, 0.8, -0.16, 0, true, 0.95),
            new DriveTimeCommand(drivetrainSubsystem, 0.45, -0, 0, true, 1.2),
            new DriveToWaypointCommand(drivetrainSubsystem, RobotMap.GyroSensor, waypoint1, 0),
            pickupGamePiece(Direction.LEFT),
            // new DriveTimeCommand(drivetrainSubsystem, -0.8, -0.1, 0, true, 1.25),
            // new DriveTimeCommand(drivetrainSubsystem, -0.25, 0, 0, true, 2.3),
            Commands.parallel(
                new DriveToWaypointCommand(drivetrainSubsystem, RobotMap.GyroSensor, waypoint2, 0),
                new WaitCommand(2.5).andThen(armCommands.highCubeCommand())
            ),
            new DriveTimeCommand(drivetrainSubsystem, 0.4, -0, 0, false, 0.7),
            new WaitCommand(0.1),
            // new DriveTimeCommand(drivetrainSubsystem, 0.5, 0, 0, false, 0.35),
            new UngrabCommand(grabSubsystem),
            new WaitCommand(0.3),
            armCommands.homeCommand()
        );
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
