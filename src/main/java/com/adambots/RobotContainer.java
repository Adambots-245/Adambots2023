
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.adambots;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import com.adambots.Constants.AutoConstants;
import com.adambots.Constants.DriveConstants;
import com.adambots.Constants.GrabbyConstants;
import com.adambots.Gamepad.Buttons;
// import com.adambots.commands.*;
// import com.adambots.commands.autonCommands.*;
import com.adambots.subsystems.*;
import com.adambots.utils.Log;
import com.adambots.utils.MockCancoder;
import com.adambots.utils.MockDoubleSolenoid;
import com.adambots.utils.MockMotor;
import com.adambots.utils.MockPhotoEye;

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
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.adambots.commands.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  // subsystems
  // private final TankDriveTrainSubsystem driveTrainSubsystem = new
  // TankDriveTrainSubsystem(RobotMap.GyroSensor,
  // RobotMap.GearShifter,
  // RobotMap.FrontRightMotor,
  // RobotMap.FrontLeftMotor,
  // RobotMap.BackLeftMotor,
  // RobotMap.BackRightMotor);

  private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem(RobotMap.frontLeftSwerveModule,
  RobotMap.rearLeftSwerveModule, RobotMap.frontRightSwerveModule, RobotMap.rearRightSwerveModule);
  // private final ArmAndGrabbySubystem armAndGrabbySubystem = new ArmAndGrabbySubystem(RobotMap.grabby, /*RobotMap.rightGrabby,*/ RobotMap.armLifter, RobotMap.leftArmExtender, RobotMap.rightArmExtender, RobotMap.rightArmPhotoEye, RobotMap.leftArmPhotoEye, RobotMap.armRotationEncoder);
  private final GrabbySubsystem grabbysubsystem = new GrabbySubsystem(RobotMap.armLifter, RobotMap.firstArmExtender, RobotMap.secondArmExtender, RobotMap.armRotationEncoder, RobotMap.grabby, RobotMap.secondExtenderPhotoEye, RobotMap.firstExtenderPhotoEye);
  // commands
  // private SequentialCommandGroup autonDriveForwardGyroDistanceCommand;

  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

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

    //Buttons.secondaryDPadE.onTrue(new CloseGrabbyCommand(armAndGrabbySubystem));
    //Buttons.secondaryDPadW.onTrue(new OpenGrabbyCommand(armAndGrabbySubystem));
    
    // Buttons.secondaryRB.whileTrue(new ExtendArmCommand(armAndGrabbySubystem, 50));
    // Buttons.secondaryLB.whileTrue(new RetractArmCommand(armAndGrabbySubystem, 50));
    // Buttons.secondaryYButton.whileTrue(new LiftArmCommand(armAndGrabbySubystem, 50));
    // Buttons.secondaryAButton.whileTrue(new LowerArmCommand(armAndGrabbySubystem, 50));
    //  Buttons.secondaryXButton.onTrue(new SetArmHomeCommand(armAndGrabbySubystem));
    
    // Buttons.secondaryBButton.onTrue(new SetArmMidCubeCommand(armAndGrabbySubystem));
    // Buttons.secondaryAButton.onTrue(new SetArmHomeCommand(armAndGrabbySubystem));
    // Buttons.secondaryYButton.onTrue(new SetArmHighCubeCommand(armAndGrabbySubystem));
    // Buttons.secondaryDPadN.onTrue(new SetArmHighConeCommand(armAndGrabbySubystem));
    // Buttons.secondaryDPadS.onTrue(new SetArmMidConeCommand(armAndGrabbySubystem));
    // Buttons.secondaryStartButton.onTrue(new SetArmInitCommand(armAndGrabbySubystem));

    // MockCancoder armCancoder = new MockCancoder(GrabbyConstants.initiaLifterValue); // + GrabbyConstants.mech2dAdjustment);
    // GrabberSubsystem grabbysubsystem = new GrabberSubsystem(new MockMotor(armCancoder), new MockMotor(), new MockMotor(), armCancoder, new MockDoubleSolenoid(), new MockPhotoEye(), new MockPhotoEye());

    Buttons.primaryDPadN.whileTrue(new InstantCommand(() -> {
      grabbysubsystem.armUp();
    }));
    
    Buttons.primaryDPadS.whileTrue(new InstantCommand(() -> {
      grabbysubsystem.armDown();
      // subsystem.setPosition(subsystem.groundPosition);
    }));

    Buttons.primaryRB.whileTrue(new InstantCommand(() -> {
      grabbysubsystem.extendFirstStage();
    }));

    Buttons.primaryLB.whileTrue(new InstantCommand(() -> {
      System.out.println("Retract First Stage");
      grabbysubsystem.retractFirstStage();
    }));

    Buttons.primaryRightStickButton.onTrue(new InstantCommand(() -> {
      System.out.println("Extend Second Stage");

      grabbysubsystem.extendSecondStage();
    }));

    Buttons.primaryLeftStickButton.onTrue(new InstantCommand(() -> {
      System.out.println("Retract Second Stage");

      grabbysubsystem.retractSecondStage();
    }));

    Buttons.primaryDPadW.onTrue(new InstantCommand(() -> {
      grabbysubsystem.openGrabby();
    }));
    
    Buttons.primaryDPadE.onTrue(new InstantCommand(() -> {
      grabbysubsystem.closeGrabby();
    }));
    
    Buttons.primaryBButton.onTrue(new InstantCommand(() -> {
      grabbysubsystem.setPosition(grabbysubsystem.midCubePosition);
    }));
    
    Buttons.primaryYButton.onTrue(new InstantCommand(() -> {
      grabbysubsystem.setPosition(grabbysubsystem.highCubePosition);
    }));

    Buttons.primaryXButton.onTrue(new InstantCommand(() -> {
      grabbysubsystem.setPosition(grabbysubsystem.midConePosition);
    }));
    
    Buttons.primaryAButton.onTrue(new InstantCommand(() -> {
      grabbysubsystem.setPosition(grabbysubsystem.highConePosition);
    }));

    Buttons.primaryBackButton.onTrue(new InstantCommand(() -> {
      grabbysubsystem.setPosition(grabbysubsystem.groundPosition);
    }));
    
    Buttons.primaryStartButton.onTrue(new InstantCommand(() -> {
      grabbysubsystem.setPosition(grabbysubsystem.homePosition);
    }));

    

    // Buttons.primaryAButton.onTrue(command);
    // Buttons.secondaryDPadE.onTrue(command);

    // Buttons.primaryBackButton.whileTrue(command);

    // Buttons.primaryAButton.onTrue(new RunCommand(() -> System.out.println("1A Pressed..."), drivetrainSubsystem));
    // Buttons.secondaryAButton.onTrue(new RunCommand(() -> System.out.println("2A Pressed..."), drivetrainSubsystem));

    // Buttons.JoystickButton1.onTrue(new RunCommand(() -> System.out.println("1 Pressed..."), drivetrainSubsystem));
    // Buttons.JoystickButton2.onTrue(new RunCommand(() -> System.out.println("2 Pressed..."), drivetrainSubsystem));
    // Buttons.JoystickThumbUp.onTrue(new RunCommand(() -> System.out.println("Up Pressed..."), drivetrainSubsystem));
  }

  private void setupDashboard() {
    // autoChooser.setDefaultOption("None", null);
    // autoChooser.addOption("Auton1Ball", new Auton1Ball(catapultSubsystem,
    // driveTrainSubsystem));
    // autoChooser.addOption("Auton2Ball", new Auton2Ball(driveTrainSubsystem,
    // intakeSubsystem, catapultSubsystem));
    // autoChooser.addOption("Position1Auton3Ball", new
    // Position1Auton3Ball(driveTrainSubsystem, intakeSubsystem,
    // catapultSubsystem));
    // autoChooser.addOption("Position1Auton5Ball", new
    // Position1Auton5Ball(driveTrainSubsystem, intakeSubsystem,
    // catapultSubsystem));
    // autoChooser.addOption("Position2Auton4Ball", new
    // Position2Auton4Ball(driveTrainSubsystem, intakeSubsystem,
    // catapultSubsystem));

    SmartDashboard.putData("Auton Mode", autoChooser);
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
  }

  private void setupDefaultCommands() {

    drivetrainSubsystem.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> drivetrainSubsystem.drive(
                -Buttons.forwardSupplier.getAsDouble(),
                -Buttons.sidewaysSupplier.getAsDouble(),
                -Buttons.rotateSupplier.getAsDouble(),
                false),
            drivetrainSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   * 
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    RobotMap.GyroSensor.reset();
    // if (autoChooser.getSelected() != null)
    // Log.info("Chosen Auton Command: ", autoChooser.getSelected().toString());
    // else
    // Log.info("Chosen Auton Command: None");

    // return autoChooser.getSelected();

    // return new LowerIntakeArmCommand(intakeSubsystem)
    // .andThen(new WaitCommand(4))
    // .andThen(new TurnToAngleFromCameraCommand(driveTrainSubsystem))

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

    String trajectoryJSON = "Unnamed.wpilib.json";
    Trajectory exampleTrajectory = new Trajectory();

    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      exampleTrajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    }

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        drivetrainSubsystem::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        drivetrainSubsystem::setModuleStates,
        drivetrainSubsystem);

    // Reset odometry to the starting pose of the trajectory.
    drivetrainSubsystem.resetOdometry(exampleTrajectory.getInitialPose());

    var field = new Field2d();
    SmartDashboard.putData("Field", field);

    // Run the "Glass" program and then choose NetworkTables -> SmartDashboard -> Field2d to view the Field.
    // The field image for 2023 is in utils folder
    field.getObject("traj").setTrajectory(exampleTrajectory);
    
    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> drivetrainSubsystem.drive(0, 0, 0, false));
  }
}
