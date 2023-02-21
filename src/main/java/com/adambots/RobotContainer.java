
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.adambots;

import java.util.List;

import javax.sound.sampled.SourceDataLine;

import java.io.IOException;
import java.nio.file.Path;
import com.adambots.Constants.AutoConstants;
import com.adambots.Constants.DriveConstants;
import com.adambots.Constants.GrabbyConstants;
import com.adambots.Gamepad.Buttons;
import com.adambots.commands.*;
import com.adambots.commands.autonCommands.*;
import com.adambots.sensors.Gyro;
import com.adambots.subsystems.*;
import com.adambots.utils.Functions;
import com.adambots.utils.Log;
import com.adambots.utils.VisionHelpers;

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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
  private final FirstExtenderSubsystem firstExtenderSubsystem = new FirstExtenderSubsystem(RobotMap.firstArmExtender, RobotMap.firstExtenderPhotoEye);
  private final SecondExtenderSubsystem secondExtenderSubsystem = new SecondExtenderSubsystem(RobotMap.secondArmExtender, RobotMap.secondExtenderPhotoEye);
  private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem(RobotMap.swerveModules, RobotMap.GyroSensor);

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
    // Buttons.secondaryDPadE.onTrue(command);
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

    Buttons.primaryDPadN.whileTrue(new LiftArmCommand(grabbyLifterSubsystem));
    Buttons.primaryDPadS.whileTrue(new LowerArmCommand(grabbyLifterSubsystem));

    Buttons.primaryRB.whileTrue(new ExtendFirstStageCommand(firstExtenderSubsystem));
    Buttons.primaryLB.whileTrue(new RetractFirstStageCommand(firstExtenderSubsystem));

    Buttons.primaryRightStickButton.whileTrue(new ExtendSecondStageCommand(secondExtenderSubsystem));
    Buttons.primaryLeftStickButton.whileTrue(new RetractSecondStageCommand(secondExtenderSubsystem));

    Buttons.JoystickButton1.onTrue(new GrabCommand(grabSubsystem));
    Buttons.primaryDPadE.onTrue(new UngrabCommand(grabSubsystem));
    Buttons.primaryDPadW.onTrue(new UngrabCommand(grabSubsystem));
    
    Buttons.primaryAButton.onTrue(Commands.parallel(new ArmLifterChangeStateCommand(grabbyLifterSubsystem, GrabbyConstants.midCubeState), new FirstExtenderChangeStateCommand(firstExtenderSubsystem, GrabbyConstants.midCubeState), new SecondExtenderChangeStateCommand(secondExtenderSubsystem, GrabbyConstants.midCubeState)));
    Buttons.primaryXButton.onTrue(Commands.parallel(new ArmLifterChangeStateCommand(grabbyLifterSubsystem, GrabbyConstants.highCubeState), new FirstExtenderChangeStateCommand(firstExtenderSubsystem, GrabbyConstants.highCubeState), new SecondExtenderChangeStateCommand(secondExtenderSubsystem, GrabbyConstants.highCubeState)));

    Buttons.primaryBButton.onTrue(Commands.parallel(new ArmLifterChangeStateCommand(grabbyLifterSubsystem, GrabbyConstants.midConeState), new FirstExtenderChangeStateCommand(firstExtenderSubsystem, GrabbyConstants.midConeState), new SecondExtenderChangeStateCommand(secondExtenderSubsystem, GrabbyConstants.midConeState)));
    Buttons.primaryYButton.onTrue(Commands.parallel(new ArmLifterChangeStateCommand(grabbyLifterSubsystem, GrabbyConstants.highConeState), new FirstExtenderChangeStateCommand(firstExtenderSubsystem, GrabbyConstants.highConeState), new SecondExtenderChangeStateCommand(secondExtenderSubsystem, GrabbyConstants.highConeState)));

    Buttons.primaryBackButton.onTrue(Commands.parallel(new ArmLifterChangeStateCommand(grabbyLifterSubsystem, GrabbyConstants.groundState), new FirstExtenderChangeStateCommand(firstExtenderSubsystem, GrabbyConstants.groundState), new SecondExtenderChangeStateCommand(secondExtenderSubsystem, GrabbyConstants.groundState)));
    Buttons.primaryStartButton.onTrue(Commands.parallel(new ArmLifterChangeStateCommand(grabbyLifterSubsystem, GrabbyConstants.initState), new FirstExtenderChangeStateCommand(firstExtenderSubsystem, GrabbyConstants.initState), new SecondExtenderChangeStateCommand(secondExtenderSubsystem, GrabbyConstants.initState)));

    Buttons.JoystickButton9.onTrue(new HockeyStopCommand(drivetrainSubsystem));
    Buttons.JoystickButton11.onTrue(new TestAutoBalanceCommand(drivetrainSubsystem, RobotMap.GyroSensor).andThen(new HockeyStopCommand(drivetrainSubsystem)));
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
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   * 
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // if (autoChooser.getSelected() != null)
    // Log.info("Chosen Auton Command: ", autoChooser.getSelected().toString());
    // else
    // Log.info("Chosen Auton Command: None");

    // return autoChooser.getSelected();

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

    RobotMap.GyroSensor.reset();

    Trajectory traj1 = Functions.getTrajectory("TopConeCubeCharge1.wpilib.json");
    SwerveControllerCommand command1 = Functions.CreateSwerveControllerCommand(drivetrainSubsystem, traj1);

    Trajectory traj2 = Functions.getTrajectory("TopConeCubeCharge2.wpilib.json");
    SwerveControllerCommand command2 = Functions.CreateSwerveControllerCommand(drivetrainSubsystem, traj2);


    // Run the "Glass" program and then choose NetworkTables -> SmartDashboard -> Field2d to view the Field.
    // The field image for 2023 is in utils folder
    
    // Run path following command, then stop at the end.
    drivetrainSubsystem.resetOdometry(traj1.getInitialPose());
    Constants.DriveConstants.field.getObject("traj").setTrajectory(traj1);

    return Commands.parallel(new ArmLifterChangeStateCommand(grabbyLifterSubsystem, GrabbyConstants.highCubeState), new FirstExtenderChangeStateCommand(firstExtenderSubsystem, GrabbyConstants.highCubeState), new SecondExtenderChangeStateCommand(secondExtenderSubsystem, GrabbyConstants.highCubeState))
    .andThen(new WaitCommand(1.7))
    .andThen(new UngrabCommand(grabSubsystem))
    .andThen(new WaitCommand(0.3))
    .andThen(Commands.parallel(new FirstExtenderChangeStateCommand(firstExtenderSubsystem, GrabbyConstants.groundState), new SecondExtenderChangeStateCommand(secondExtenderSubsystem, GrabbyConstants.groundState)))
    .andThen(Commands.parallel(command1, new WaitCommand(1).andThen(new ArmLifterChangeStateCommand(grabbyLifterSubsystem, GrabbyConstants.groundState))))
    .andThen(new AutonPickupCommand(drivetrainSubsystem, grabSubsystem, 0.8))
    .andThen(new WaitCommand(0.2))

    .andThen(Commands.parallel(new ArmLifterChangeStateCommand(grabbyLifterSubsystem, GrabbyConstants.balancingState), new FirstExtenderChangeStateCommand(firstExtenderSubsystem, GrabbyConstants.balancingState), new SecondExtenderChangeStateCommand(secondExtenderSubsystem, GrabbyConstants.balancingState)))
    // .andThen(() -> drivetrainSubsystem.resetOdometry(traj2.getInitialPose()))
    .andThen(command2)
    .andThen(new TestAutoBalanceCommand(drivetrainSubsystem, RobotMap.GyroSensor)
    .andThen(new HockeyStopCommand(drivetrainSubsystem)))
    .andThen(() -> drivetrainSubsystem.stop());

    // return new TestDriveToAprilTagCommand(drivetrainSubsystem, (int)VisionHelpers.getDetectedResult(), RobotMap.GyroSensor);
  }
}
