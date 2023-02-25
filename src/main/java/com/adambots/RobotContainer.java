
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.adambots;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

import javax.sound.sampled.SourceDataLine;

import java.io.IOException;
import java.nio.file.Path;
import com.adambots.Constants.AutoConstants;
import com.adambots.Constants.DriveConstants;
import com.adambots.Constants.GrabbyConstants;
import com.adambots.Gamepad.Buttons;
import com.adambots.Vision.VisionHelpers;
import com.adambots.commands.*;
import com.adambots.commands.autonCommands.*;
import com.adambots.commands.autonCommands.autonCommandGroups.TopCubeCubeCharge;
import com.adambots.commands.autonCommands.autonCommandGroups.TopCubeCubeScore;
import com.adambots.sensors.Gyro;
import com.adambots.subsystems.*;
import com.adambots.utils.Functions;
import com.adambots.utils.Log;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final GrabSubsystem grabSubsystem = new GrabSubsystem(RobotMap.grabby);
  private final GrabbyLifterSubsystem grabbyLifterSubsystem = new GrabbyLifterSubsystem(RobotMap.armLifter, RobotMap.armRotationEncoder);
  private final FirstExtenderSubsystem firstExtenderSubsystem = new FirstExtenderSubsystem(RobotMap.firstArmExtender, RobotMap.firstExtenderPhotoEye, RobotMap.armRotationEncoder);
  private final SecondExtenderSubsystem secondExtenderSubsystem = new SecondExtenderSubsystem(RobotMap.secondArmExtender, RobotMap.secondExtenderPhotoEye, RobotMap.armRotationEncoder);
  private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem(RobotMap.swerveModules, RobotMap.GyroSensor);
  Collection<TalonFX> alist = new ArrayList<>(){{
    add(RobotMap.firstArmExtender);
    add(RobotMap.secondArmExtender);
    add(RobotMap.armLifter);
  }};
  String[] songs = {"song1.chrp", "song2.chrp", "song3.chrp", "song4.chrp", "song5.chrp", "song6.chrp"};
  private final MusicSubsystem musicSubsystem = new MusicSubsystem(alist, songs);
  private int idx = 0;

  // commands
  // private SequentialCommandGroup autonDriveForwardGyroDistanceCommand;

  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  private SlewRateLimiter slewFilter;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    setupDefaultCommands();

    // Don't enable this for the competition
    // Log.saveToFile("/home/lvuser/robot.txt");
    Log.info("Starting Robot Container ...");

    // Configure the button bindings
    configureButtonBindings();

    // configure the dashboard
    setupDashboard();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // Buttons.primaryAButton.onTrue(new ChangeStream());

    // MockCancoder armCancoder = new MockCancoder(GrabbyConstants.initiaLifterValue); // + GrabbyConstants.mech2dAdjustment);
    // GrabberSubsystem grabbysubsystem = new GrabberSubsystem(new MockMotor(armCancoder), new MockMotor(), new MockMotor(), armCancoder, new MockDoubleSolenoid(), new MockPhotoEye(), new MockPhotoEye());

    ArmCommands armCommands = new ArmCommands(firstExtenderSubsystem, secondExtenderSubsystem, grabbyLifterSubsystem, grabSubsystem);

    Buttons.primaryStartButton.onTrue(new RunCommand(() -> {
      if (idx > 10){
        idx = 0;
      }
      
      musicSubsystem.playMusic(idx++);
    }, musicSubsystem));


    Buttons.primaryDPadN.whileTrue(armCommands.LiftArmCommand);
    Buttons.primaryDPadS.whileTrue(armCommands.LowerArmCommand);


    // Buttons.primaryRB.whileTrue(armCommands.ExtendFirstStageCommand);
    // Buttons.primaryLB.whileTrue(armCommands.RetractFirstStageCommand);

    // Buttons.primaryRightStickButton.whileTrue(armCommands.ExtendSecondStageCommand);
    // Buttons.primaryLeftStickButton.whileTrue(armCommands.RetractSecondStageCommand);

    Buttons.primaryRightStickButton.whileTrue(armCommands.SmartExtendArmCommand);
    Buttons.primaryLeftStickButton.whileTrue(armCommands.SmartRetractArmCommand);

    Buttons.JoystickButton1.onTrue(armCommands.GrabCommand);

    // Buttons.primaryDPadE.onTrue(armCommands.UngrabWithRetractCommand);
    // Buttons.primaryDPadW.onTrue(armCommands.UngrabWithRetractCommand);

    // Buttons.primaryDPadE.onTrue(armCommands.UngrabWithRetractCommand);
    Buttons.primaryDPadW.onTrue(armCommands.UngrabWithRetractCommand);
    
    Buttons.primaryBackButton.onTrue(armCommands.MidCubeCommand);
    Buttons.primaryXButton.onTrue(armCommands.HighCubeCommand);

    // Buttons.primaryStartButton.onTrue(armCommands.MidConeCommand);
    Buttons.primaryYButton.onTrue(armCommands.HighConeCommand);

    // Buttons.primaryBackButton.onTrue(armCommands.GroundCommand);
    // Buttons.primaryStartButton.onTrue(armCommands.HomeCommand);

    Buttons.primaryAButton.onTrue(armCommands.GroundCommand);
    Buttons.primaryBButton.onTrue(armCommands.HomeCommand);

    Buttons.JoystickButton9.onTrue(new HockeyStopCommand(drivetrainSubsystem));
    // Buttons.JoystickButton11.onTrue(new TestAutoBalanceCommand(drivetrainSubsystem, RobotMap.GyroSensor).andThen(new HockeyStopCommand(drivetrainSubsystem)));
    // Buttons.JoystickButton3.onTrue(new DriveToAprilTagCommand(drivetrainSubsystem, VisionHelpers.getAprilTagPose2d(), (int)VisionHelpers.getDetectedResult(), RobotMap.GyroSensor));
  }

  private void setupDashboard() {
    autoChooser.setDefaultOption("None", null);

    autoChooser.addOption("BlueTopCubeCubeCharge",
      new TopCubeCubeCharge(
      Functions.getTrajectory("BlueTopCubeCube1.wpilib.json"), 
      Functions.getTrajectory("BlueTopCubeCubeCharge2.wpilib.json"), 
      drivetrainSubsystem, grabbyLifterSubsystem, firstExtenderSubsystem, secondExtenderSubsystem, grabSubsystem)
    );

    autoChooser.addOption("BlueTopCubeCubeScore",
      new TopCubeCubeScore(
      Functions.getTrajectory("BlueTopCubeCube1.wpilib.json"), 
      Functions.getTrajectory("BlueTopCubeCubeScore2.wpilib.json"), 
      drivetrainSubsystem, grabbyLifterSubsystem, firstExtenderSubsystem, secondExtenderSubsystem, grabSubsystem)
    );

    SmartDashboard.putData("Auton Mode", autoChooser);
    slewFilter  = new SlewRateLimiter(70);
  }

  /**
   * This method will be called periodically to update the dashboard values
   * Call from Robot.java robotPeriodic
   */
  public void updateDashboard() {
    SmartDashboard.putNumber("getY", Buttons.forwardSupplier.getAsDouble());
    SmartDashboard.putNumber("getX", Buttons.sidewaysSupplier.getAsDouble());
    SmartDashboard.putNumber("getZ", Buttons.rotateSupplier.getAsDouble());
    SmartDashboard.putNumber("pitch", RobotMap.GyroSensor.getPitch());
    SmartDashboard.putNumber("roll", RobotMap.GyroSensor.getRoll());

    SmartDashboard.putData("Field", Constants.DriveConstants.field);

    SmartDashboard.putNumber("Normal:" , Buttons.forwardSupplier.getAsDouble());
    SmartDashboard.putNumber("Vision X:" , VisionHelpers.getAprilTagPose2d().getX());
    SmartDashboard.putNumber("Vision Y:" , VisionHelpers.getAprilTagPose2d().getY());
    SmartDashboard.putNumber("Vision Index:" , VisionHelpers.getDetectedResult());
    SmartDashboard.putNumber("Curve2:" , slewFilter.calculate(Buttons.forwardSupplier.getAsDouble()));
    SmartDashboard.putNumber("Sigmoid:" , Buttons.smoothInput(Buttons.forwardSupplier.getAsDouble()));
  }

  private void setupDefaultCommands() {

    drivetrainSubsystem.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> drivetrainSubsystem.drive( //ADDED CURVES TO Buttons, CHANGE BACK IF IT DOESNT WORK - TOMMY
                -Buttons.forwardSupplier.getAsDouble(),
                -Buttons.sidewaysSupplier.getAsDouble(),
                -Buttons.rotateSupplier.getAsDouble(),
                false),
            drivetrainSubsystem));

          new LiftArmCommand(grabbyLifterSubsystem, Buttons.primaryJoystick.getRightTriggerAxis() * 10);
          new LowerArmCommand(grabbyLifterSubsystem, Buttons.primaryJoystick.getLeftTriggerAxis() * 10);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   * 
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    if (autoChooser.getSelected() != null) {
      Log.info("Chosen Auton Command: ", autoChooser.getSelected().toString());
    } else {
      Log.info("Chosen Auton Command: None");
    }
    return autoChooser.getSelected();

    /*
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(3, 0)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0.5, new Rotation2d(0)),
        config); 
    */
    // System.out.println("Total time: " + exampleTrajectory.getTotalTimeSeconds());

    // return new AutonLeftRedPlaceCubeGrabCharge(
    // Functions.getTrajectory("TopConeCubeCharge1.wpilib.json"), 
    // Functions.getTrajectory("TopConeCubeCharge2.wpilib.json"), 
    // drivetrainSubsystem, grabbyLifterSubsystem, firstExtenderSubsystem, secondExtenderSubsystem, grabSubsystem);

    // return new TestDriveToAprilTagCommand(drivetrainSubsystem, (int)VisionHelpers.getDetectedResult(), RobotMap.GyroSensor);
    //Tardirades can survive in a vacuum
  }
}
