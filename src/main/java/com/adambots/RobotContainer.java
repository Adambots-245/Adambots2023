
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.adambots;

import com.adambots.Gamepad.Buttons;
import com.adambots.commands.ArmCommands;
import com.adambots.commands.autonCommands.AutonCommands;
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
  private final GrabSubsystem grabSubsystem = new GrabSubsystem(RobotMap.grabby);
  private final FirstExtenderSubsystem firstExtenderSubsystem = new FirstExtenderSubsystem(RobotMap.firstArmExtender, RobotMap.firstExtenderPhotoEye, RobotMap.armRotationEncoder);
  private final SecondExtenderSubsystem secondExtenderSubsystem = new SecondExtenderSubsystem(RobotMap.secondArmExtender, RobotMap.secondExtenderPhotoEye, RobotMap.armRotationEncoder);
  private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem(RobotMap.swerveModules, RobotMap.GyroSensor);
  private final ArmCommands armCommands = new ArmCommands(firstExtenderSubsystem, secondExtenderSubsystem, grabbyLifterSubsystem, grabSubsystem);
  private final AutonCommands autonCommands  = new AutonCommands(grabSubsystem, grabbyLifterSubsystem, drivetrainSubsystem, armCommands, firstExtenderSubsystem);

  //Creates a SmartDashboard element to allow drivers to select differnt autons
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
    //XBox Controller Keybinds
    Buttons.primaryBackButton.onTrue(armCommands.groundCommand());
    Buttons.primaryBackButton.onFalse(armCommands.homeCommand());

    Buttons.primaryStartButton.onTrue(armCommands.homeCommand());

    Buttons.primaryRB.whileTrue(armCommands.smartExtendArmCommand());
    Buttons.primaryLB.whileTrue(armCommands.smartRetractArmCommand());

    Buttons.primaryLeftStickButton.whileTrue(armCommands.ungrabWithRetractCommand());
    Buttons.primaryRightStickButton.whileTrue(armCommands.ungrabWithRetractCommand());

    Buttons.primaryDPadNE.whileTrue(armCommands.liftArmCommand()); //Allows for slightly imprecise input on DPad
    Buttons.primaryDPadN.whileTrue(armCommands.liftArmCommand());
    Buttons.primaryDPadNW.whileTrue(armCommands.liftArmCommand());

    Buttons.primaryDPadSE.whileTrue(armCommands.lowerArmCommand());
    Buttons.primaryDPadS.whileTrue(armCommands.lowerArmCommand());
    Buttons.primaryDPadSW.whileTrue(armCommands.lowerArmCommand());

    Buttons.primaryAButton.onTrue(armCommands.midCubeCommand());
    Buttons.primaryXButton.onTrue(armCommands.highCubeCommand());

    Buttons.primaryBButton.onTrue(armCommands.midConeCommand());
    Buttons.primaryYButton.onTrue(armCommands.highConeCommand());
    
    //Joystick Keybinds
    Buttons.JoystickButton1.onTrue(armCommands.grabCommand());
    Buttons.JoystickButton4.onTrue(autonCommands.humanPlayerClamp());

    Buttons.JoystickButton3.onTrue(autonCommands.humanStationConePickup());
    Buttons.JoystickButton2.onTrue(autonCommands.humanStationBackConePickup());
    Buttons.JoystickButton7.onTrue(autonCommands.groundConePickup());
    Buttons.JoystickButton5.onTrue(autonCommands.autoInitAndScoreCone());

    Buttons.JoystickButton6.onTrue(autonCommands.resetGyroCommand());

    //Debugging and Testing
    // Buttons.JoystickButton4.onTrue(armCommands.humanStationConeCommand());
    // Buttons.JoystickButton11.onTrue(Commands.deadline(new WaitCommand(1.5), autonCommands.driveTillBumpedCommand()));
    // Buttons.JoystickButton16.onTrue(new TurnToGamePieceCommand(drivetrainSubsystem, RobotMap.lidar, Direction.RIGHT));
    // Buttons.JoystickButton16.onTrue(autonCommands.autoInitAndScoreCone());

    // Buttons.JoystickButton16.onTrue(new TestAutoBalanceCommand(drivetrainSubsystem, RobotMap.GyroSensor, grabbyLifterSubsystem).andThen(new HockeyStopCommand(drivetrainSubsystem)));
    // Buttons.JoystickButton16.onTrue(new AutoBalanceCommand(drivetrainSubsystem, RobotMap.GyroSensor, grabbyLifterSubsystem));
  }

  private void setupDashboard() {    
    Command defaultCommand = autonCommands.autoInitAndScoreCone(); //Have a default setup so even if drivers forget to select an auton, preloaded objects are still scored

    autoChooser.setDefaultOption("CHOOSE AN AUTON", defaultCommand);
    autoChooser.addOption("MidChargeStation", autonCommands.midCubeCharge());
    autoChooser.addOption("BLUE TopSimple", autonCommands.scorePickupTopBlue());
    autoChooser.addOption("RED TopSimple", autonCommands.scorePickupTopRed());
    autoChooser.addOption("BLUE BottomSimple", autonCommands.scorePickupBottomBlue());
    autoChooser.addOption("RED BottomSimple", autonCommands.scorePickupBottomRed());

    //Adds various data to the dashboard that is useful for driving and debugging
    SmartDashboard.putData("Auton Mode", autoChooser);
    SmartDashboard.putData("Field", Constants.DriveConstants.field);

    // Dash.add("Port 0 Current", () -> RobotMap.PDM.getCurrent(0));
    // Dash.add("Port 1 Current", () -> RobotMap.PDM.getCurrent(1));
    // Dash.add("Port 11 Current", () -> RobotMap.PDM.getCurrent(11));
    // Dash.add("Port 14 Current", () -> RobotMap.PDM.getCurrent(14));

    Dash.add("getY", Buttons.forwardSupplier);
    Dash.add("getX", Buttons.sidewaysSupplier);
    Dash.add("getZ", Buttons.rotateSupplier);
    Dash.add("getRawZ", () -> Buttons.ex3dPro.getZ());

    Dash.add("odom x", () -> drivetrainSubsystem.getPose().getX());
    Dash.add("odom y", () -> drivetrainSubsystem.getPose().getY());
    Dash.add("Bumped", () -> drivetrainSubsystem.getBumped());
    Dash.add("yaw", () -> RobotMap.GyroSensor.getAngle());
    Dash.add("pitch", () -> RobotMap.GyroSensor.getPitch());
    Dash.add("roll", () -> RobotMap.GyroSensor.getRoll());

    Dash.add("LIDAR Dist", () -> RobotMap.lidar.getInches());

    // // Dash.add("Vision X:" , () -> VisionHelpers.getAprilTagPose2d().getX());
    // // Dash.add("Vision Y:" , () -> VisionHelpers.getAprilTagPose2d().getY());
    // // Dash.add("Vision Index:" , () -> VisionHelpers.getDetectedResult());

    // Dash.add("isDetectingPieces", () -> VisionHelpers.isDetectingPieces("cube"));
    // Dash.add("pieceX", () -> VisionHelpers.getPieceX("cube"));
    // Dash.add("pieceY", () -> VisionHelpers.getPieceY("cube"));
    // Dash.add("DistanceToObject", () -> VisionHelpers.getDistanceToObject());
    // Dash.add("tx", () -> VisionHelpers.getTX());
    // Dash.add("Aligned", () -> VisionHelpers.isAligned());
  }

  private void setupDefaultCommands() {
    drivetrainSubsystem.setDefaultCommand(
        new RunCommand(
            () -> drivetrainSubsystem.drive(
                Buttons.forwardSupplier.getAsDouble()*1.65,
                Buttons.sidewaysSupplier.getAsDouble()*1.65,
                -Buttons.rotateSupplier.getAsDouble()*1.43,
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