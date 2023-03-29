
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.adambots;

import com.adambots.Gamepad.Buttons;
import com.adambots.Vision.VisionHelpers;
import com.adambots.commands.ArmCommands;
import com.adambots.commands.RotateDownGrabbyCommand;
import com.adambots.commands.RotateUpGrabbyCommand;
import com.adambots.commands.autonCommands.AutonCommands;
import com.adambots.commands.autonCommands.HockeyStopCommand;
import com.adambots.subsystems.DrivetrainSubsystem;
import com.adambots.subsystems.FirstExtenderSubsystem;
import com.adambots.subsystems.GrabSubsystem;
import com.adambots.subsystems.GrabbyLifterSubsystem;
import com.adambots.subsystems.SecondExtenderSubsystem;
import com.adambots.utils.Dash;
import com.adambots.utils.Log;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final GrabbyLifterSubsystem grabbyLifterSubsystem = new GrabbyLifterSubsystem(RobotMap.armLifter, RobotMap.armRotationEncoder);
  private final GrabSubsystem grabSubsystem = new GrabSubsystem(RobotMap.grabby, RobotMap.grabbyTiltMotor, RobotMap.armRotationEncoder, RobotMap.lidar);
  private final FirstExtenderSubsystem firstExtenderSubsystem = new FirstExtenderSubsystem(RobotMap.firstArmExtender, RobotMap.firstExtenderPhotoEye, RobotMap.armRotationEncoder);
  private final SecondExtenderSubsystem secondExtenderSubsystem = new SecondExtenderSubsystem(RobotMap.secondArmExtender, RobotMap.secondExtenderPhotoEye, RobotMap.armRotationEncoder);
  private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem(RobotMap.swerveModules, RobotMap.GyroSensor);
  private final ArmCommands armCommands = new ArmCommands(firstExtenderSubsystem, secondExtenderSubsystem, grabbyLifterSubsystem, grabSubsystem);
  private final AutonCommands autonCommands  = new AutonCommands(grabSubsystem, grabbyLifterSubsystem, firstExtenderSubsystem, secondExtenderSubsystem, drivetrainSubsystem, armCommands);


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
   * 
   */
  
  private void configureButtonBindings() {
    //Enable for XBox controller code
    Buttons.primaryBackButton.onTrue(armCommands.groundCommand());
    Buttons.primaryBackButton.onFalse(armCommands.homeCommand());

    Buttons.primaryStartButton.onTrue(armCommands.homeCommand());

    Buttons.primaryRB.whileTrue(armCommands.smartExtendArmCommand());
    Buttons.primaryLB.whileTrue(armCommands.smartRetractArmCommand());

    Buttons.primaryLeftStickButton.whileTrue(armCommands.ungrabWithRetractCommand());
    Buttons.primaryRightStickButton.whileTrue(armCommands.ungrabWithRetractCommand());

    Buttons.primaryDPadN.whileTrue(armCommands.liftArmCommand());
    Buttons.primaryDPadS.whileTrue(armCommands.lowerArmCommand());
    Buttons.primaryDPadNE.whileTrue(armCommands.liftArmCommand());
    Buttons.primaryDPadSE.whileTrue(armCommands.lowerArmCommand());
    Buttons.primaryDPadNW.whileTrue(armCommands.liftArmCommand());
    Buttons.primaryDPadSW.whileTrue(armCommands.lowerArmCommand());

    Buttons.rightTrigger.whileTrue(new RotateUpGrabbyCommand(grabSubsystem, RobotMap.grabbyTiltMotor));    
    Buttons.leftTrigger.whileTrue(new RotateDownGrabbyCommand(grabSubsystem, RobotMap.grabbyTiltMotor));

    Buttons.primaryAButton.onTrue(armCommands.midCubeCommand());
    Buttons.primaryXButton.onTrue(armCommands.highCubeCommand());

    Buttons.primaryBButton.onTrue(armCommands.midConeCommand());
    Buttons.primaryYButton.onTrue(armCommands.highConeCommand());
    
    //Joystick Keybinds
    Buttons.JoystickButton6.onTrue(new HockeyStopCommand(drivetrainSubsystem));
    Buttons.JoystickButton1.onTrue(armCommands.grabCommand());
    Buttons.JoystickButton3.onTrue(autonCommands.humanStationPickup());
    Buttons.JoystickButton7.onTrue(armCommands.sideStationCommand());

    Buttons.JoystickButton4.onTrue(autonCommands.resetGyroCommand());

    Buttons.JoystickButton10.onTrue(autonCommands.pickupGamePiece("cube"));
    // Buttons.JoystickButton16.onTrue(autonCommands.humanStationPickup());
    // Buttons.JoystickButton16.onTrue(new TestAutoBalanceCommand(drivetrainSubsystem, RobotMap.GyroSensor, grabbyLifterSubsystem).andThen(new HockeyStopCommand(drivetrainSubsystem)));
    // Buttons.JoystickButton7.onTrue(new AutoBalanceCommand(drivetrainSubsystem, RobotMap.GyroSensor).andThen(new HockeyStopCommand(drivetrainSubsystem)));
  }

  private void setupDashboard() {    
    Command defaultCommand = autonCommands.autoInitAndScoreCone();

    autoChooser.setDefaultOption("CHOOSE AN AUTON", defaultCommand);
    autoChooser.addOption("MidChargeStation", autonCommands.midCubeCharge());
    // // Will automatically call blue or red
    autoChooser.addOption("TopSimple", autonCommands.scorePickupTop());
    autoChooser.addOption("BottomSimple", autonCommands.scorePickupBottom());


    SmartDashboard.putData("Auton Mode", autoChooser);
    SmartDashboard.putData("Field", Constants.DriveConstants.field);

    // Dash.add("getY", Buttons.forwardSupplier);
    // Dash.add("getX", Buttons.sidewaysSupplier);
    // Dash.add("getZ", Buttons.rotateSupplier);
    // Dash.add("yaw", () -> RobotMap.GyroSensor.getAngle());
    // Dash.add("pitch", () -> RobotMap.GyroSensor.getPitch());
    // Dash.add("roll", () -> RobotMap.GyroSensor.getRoll());
    // Dash.add("Arm Encoder w/ offset", () -> RobotMap.armRotationEncoder.getAbsolutePosition()+GrabbyConstants.limitOffset);

    Dash.add("LIDAR Dist", () -> RobotMap.lidar.getInches());

    // // Dash.add("Vision X:" , () -> VisionHelpers.getAprilTagPose2d().getX());
    // // Dash.add("Vision Y:" , () -> VisionHelpers.getAprilTagPose2d().getY());
    // // Dash.add("Vision Index:" , () -> VisionHelpers.getDetectedResult());

    // Dash.add("Lidar", () -> RobotMap.lidar.getInches());

    Dash.add("isDetectingPieces", () -> VisionHelpers.isDetectingPieces("cube"));
    Dash.add("pieceX", () -> VisionHelpers.getPieceX("cube"));
    Dash.add("pieceY", () -> VisionHelpers.getPieceY("cube"));
    Dash.add("DistanceToObject", () -> VisionHelpers.getDistanceToObject());
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
                true),
            drivetrainSubsystem));
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
    //Tardirades can survive in a vacuum
  }
}
// Rudy was here