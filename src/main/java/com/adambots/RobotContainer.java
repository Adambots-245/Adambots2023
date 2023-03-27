
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.adambots;

import com.adambots.Constants.GrabbyConstants;
import com.adambots.Gamepad.Buttons;
import com.adambots.Vision.VisionHelpers;
import com.adambots.commands.ArmCommands;
import com.adambots.commands.ArmLifterChangeStateCommand;
import com.adambots.commands.DriveToDistanceCommand;
import com.adambots.commands.FirstExtenderChangeStateCommand;
import com.adambots.commands.GrabCommand;
import com.adambots.commands.SecondExtenderChangeStateCommand;
import com.adambots.commands.TurnToObjectCommand;
import com.adambots.commands.autonCommands.HockeyStopCommand;
import com.adambots.commands.autonCommands.autonCommandGroups.AutoInitAndScoreCube;
import com.adambots.commands.autonCommands.autonCommandGroups.MidCubeCharge;
import com.adambots.commands.autonCommands.autonCommandGroups.NoTrajInitAndScore;
import com.adambots.commands.autonCommands.autonCommandGroups.ScorePickupBottom;
import com.adambots.commands.autonCommands.autonCommandGroups.ScorePickupTop;
import com.adambots.subsystems.CANdleSubsystem;
import com.adambots.subsystems.DrivetrainSubsystem;
import com.adambots.subsystems.FirstExtenderSubsystem;
import com.adambots.subsystems.GrabSubsystem;
import com.adambots.subsystems.GrabbyLifterSubsystem;
import com.adambots.subsystems.SecondExtenderSubsystem;
import com.adambots.utils.Dash;
import com.adambots.utils.Functions;
import com.adambots.utils.Log;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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

  private final GrabSubsystem grabSubsystem = new GrabSubsystem(RobotMap.grabby);
  private final GrabbyLifterSubsystem grabbyLifterSubsystem = new GrabbyLifterSubsystem(RobotMap.armLifter, RobotMap.armRotationEncoder);
  private final FirstExtenderSubsystem firstExtenderSubsystem = new FirstExtenderSubsystem(RobotMap.firstArmExtender, RobotMap.firstExtenderPhotoEye, RobotMap.armRotationEncoder);
  private final SecondExtenderSubsystem secondExtenderSubsystem = new SecondExtenderSubsystem(RobotMap.secondArmExtender, RobotMap.secondExtenderPhotoEye, RobotMap.armRotationEncoder);
  private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem(RobotMap.swerveModules, RobotMap.GyroSensor);
  private final CANdleSubsystem ledSubsystem = new CANdleSubsystem(RobotMap.candleLEDs, RobotMap.ultrasonic);  

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

    // Buttons.primaryAButton.onTrue(new ChangeStream());

    // MockCancoder armCancoder = new MockCancoder(GrabbyConstants.initiaLifterValue); // + GrabbyConstants.mech2dAdjustment);
    // GrabberSubsystem grabbysubsystem = new GrabberSubsystem(new MockMotor(armCancoder), new MockMotor(), new MockMotor(), armCancoder, new MockDoubleSolenoid(), new MockPhotoEye(), new MockPhotoEye());

    ArmCommands armCommands = new ArmCommands(firstExtenderSubsystem, secondExtenderSubsystem, grabbyLifterSubsystem, grabSubsystem, ledSubsystem);

    //Enable for XBoxx controller code
    Buttons.primaryBackButton.onTrue(armCommands.GroundCommand);
    Buttons.primaryBackButton.onFalse(armCommands.HomeCommand);
    Buttons.primaryStartButton.onTrue(armCommands.HomeCommand);

    Buttons.primaryRB.whileTrue(armCommands.SmartExtendArmCommand);
    Buttons.primaryLB.whileTrue(armCommands.SmartRetractArmCommand);

    Buttons.primaryLeftStickButton.whileTrue(armCommands.UngrabWithRetractCommand);
    Buttons.primaryRightStickButton.whileTrue(armCommands.UngrabWithRetractCommand);

    Buttons.primaryDPadN.whileTrue(armCommands.LiftArmCommand);
    Buttons.primaryDPadS.whileTrue(armCommands.LowerArmCommand);
    Buttons.primaryDPadNE.whileTrue(armCommands.LiftArmCommand);
    Buttons.primaryDPadSE.whileTrue(armCommands.LowerArmCommand);
    Buttons.primaryDPadNW.whileTrue(armCommands.LiftArmCommand);
    Buttons.primaryDPadSW.whileTrue(armCommands.LowerArmCommand);


    Buttons.primaryAButton.onTrue(armCommands.MidCubeCommand);
    Buttons.primaryXButton.onTrue(armCommands.HighCubeCommand);

    Buttons.primaryBButton.onTrue(armCommands.MidConeCommand);
    Buttons.primaryYButton.onTrue(armCommands.HighConeCommand);
    
    //Joystick Keybinds
    Buttons.JoystickButton6.onTrue(new HockeyStopCommand(drivetrainSubsystem));
    Buttons.JoystickButton1.onTrue(armCommands.GrabCommand);
    Buttons.JoystickButton3.onTrue(armCommands.HumanStationCommand);
    Buttons.JoystickButton7.onTrue(armCommands.SideStationCommand);

    Buttons.JoystickButton4.onTrue(new InstantCommand(() -> RobotMap.GyroSensor.reset()));

    Buttons.JoystickButton10.onTrue(
      new TurnToObjectCommand(drivetrainSubsystem, "cube")
      .andThen(Commands.parallel(
        new ArmLifterChangeStateCommand(grabbyLifterSubsystem, GrabbyConstants.groundState),
        new FirstExtenderChangeStateCommand(firstExtenderSubsystem, GrabbyConstants.groundState),
        new SecondExtenderChangeStateCommand(secondExtenderSubsystem, GrabbyConstants.groundState)))
      .andThen(new DriveToDistanceCommand(drivetrainSubsystem, RobotMap.lidar, grabbyLifterSubsystem))
      .andThen(new GrabCommand(grabSubsystem))
      );

    // Buttons.JoystickButton7.onTrue(new TestAutoBalanceCommand(drivetrainSubsystem, RobotMap.GyroSensor).andThen(new HockeyStopCommand(drivetrainSubsystem)));
    // Buttons.JoystickButton7.onTrue(new AutoBalanceCommand(drivetrainSubsystem, RobotMap.GyroSensor).andThen(new HockeyStopCommand(drivetrainSubsystem)));



    // Trigger trigger = new Trigger(() -> {
    //   double distance = RobotMap.ultrasonic.getInches();
    //   return (distance < 34 && distance > 12);
    // });

    // trigger.onTrue(new InstantCommand(() -> {
    //   Buttons.rumble(Buttons.primaryJoystick, 1000, 1);
    //   ledSubsystem.setColor(255, 0, 0);
    //   // ledSubsystem.changeAnimation(AnimationTypes.Rainbow);
    //   RobotMap.grabbyMotor.set(0.1);
    // }));

    // trigger.onFalse(new WaitCommand(1).andThen(new InstantCommand(() -> {
    //   // ledSubsystem.changeAnimation(AnimationTypes.Empty);
    //   ledSubsystem.clearAllAnims();
    //   RobotMap.grabbyMotor.set(0);
    //   ledSubsystem.setColor(255, 216, 0);
    // })));
    // Buttons.JoystickButton11.onTrue(new TestAutoBalanceCommand(drivetrainSubsystem, RobotMap.GyroSensor).andThen(new HockeyStopCommand(drivetrainSubsystem)));
  }

  private void setupDashboard() {
    autoChooser.setDefaultOption("CHOOSE AN AUTON", 
      new NoTrajInitAndScore(
      drivetrainSubsystem, grabbyLifterSubsystem, firstExtenderSubsystem, secondExtenderSubsystem, grabSubsystem)
    );

    autoChooser.addOption("MidChargeStation",
      new MidCubeCharge(
      RobotMap.GyroSensor, drivetrainSubsystem, grabbyLifterSubsystem, firstExtenderSubsystem, secondExtenderSubsystem, grabSubsystem)
    );

    // autoChooser.addOption("BlueTopSimple", //drive for time and distance
    //   new BasicTop(
    //     drivetrainSubsystem, grabbyLifterSubsystem, firstExtenderSubsystem, secondExtenderSubsystem, grabSubsystem)
    // );
    autoChooser.addOption("BlueTopSimple",
      new ScorePickupTop(
        Functions.getTrajectory("BlueTopCubeCube1.wpilib.json"), 
        Functions.getTrajectory("BlueTopCubeCubeScore2.wpilib.json"),
        drivetrainSubsystem, grabbyLifterSubsystem, firstExtenderSubsystem, secondExtenderSubsystem, grabSubsystem)
    );
    autoChooser.addOption("BlueBottomSimple",
      new ScorePickupBottom(
      Functions.getTrajectory("BlueBottomCubeCube1.wpilib.json"),
      Functions.getTrajectory("BlueBottomCubeCubeScore2.wpilib.json"), 
      drivetrainSubsystem, grabbyLifterSubsystem, firstExtenderSubsystem, secondExtenderSubsystem, grabSubsystem)
    );

    autoChooser.addOption("RedTopSimple",
      new ScorePickupTop(
      Functions.getTrajectory("RedTopCubeCube1.wpilib.json"), 
      Functions.getTrajectory("RedTopCubeCubeScore2.wpilib.json"),
      drivetrainSubsystem, grabbyLifterSubsystem, firstExtenderSubsystem, secondExtenderSubsystem, grabSubsystem)
    );
    autoChooser.addOption("RedBottomSimple",
      new ScorePickupBottom(
      Functions.getTrajectory("RedBottomCubeCube1.wpilib.json"), 
      Functions.getTrajectory("RedBottomCubeCubeScore2.wpilib.json"),
      drivetrainSubsystem, grabbyLifterSubsystem, firstExtenderSubsystem, secondExtenderSubsystem, grabSubsystem)
    );

    SmartDashboard.putData("Auton Mode", autoChooser);
    Dash.add("getY", Buttons.forwardSupplier);
    Dash.add("getX", Buttons.sidewaysSupplier);
    Dash.add("getZ", Buttons.rotateSupplier);
    Dash.add("yaw", () -> RobotMap.GyroSensor.getAngle());
    Dash.add("pitch", () -> RobotMap.GyroSensor.getPitch());
    Dash.add("roll", () -> RobotMap.GyroSensor.getRoll());

    Dash.add("LIDAR Dist", () -> RobotMap.lidar.getDistance());

    SmartDashboard.putData("Field", Constants.DriveConstants.field);

    // Dash.add("Vision X:" , () -> VisionHelpers.getAprilTagPose2d().getX());
    // Dash.add("Vision Y:" , () -> VisionHelpers.getAprilTagPose2d().getY());
    // Dash.add("Vision Index:" , () -> VisionHelpers.getDetectedResult());

    Dash.add("Ultrasonic Distance", () -> RobotMap.ultrasonic.getInches());

    Dash.add("Lidar", () -> RobotMap.lidar.getInches());

    Dash.add("isDetectingPieces", () -> VisionHelpers.isDetectingPieces("cube"));

    Dash.add("pieceX", () -> VisionHelpers.getPieceX("cube"));
    Dash.add("pieceY", () -> VisionHelpers.getPieceY("cube"));

    Dash.add("DistanceToObject", () -> VisionHelpers.getDistanceToObject());
  }

  /**
   * This method will be called periodically to update the dashboard values
   * Call from Robot.java robotPeriodic
   */
  public void updateDashboard() {
    // SmartDashboard.putNumber("getY", Buttons.forwardSupplier.getAsDouble());
    // SmartDashboard.putNumber("getX", Buttons.sidewaysSupplier.getAsDouble());
    // SmartDashboard.putNumber("getZ", Buttons.rotateSupplier.getAsDouble());
    // SmartDashboard.putNumber("pitch", RobotMap.GyroSensor.getPitch());
    // SmartDashboard.putNumber("roll", RobotMap.GyroSensor.getRoll());

    // SmartDashboard.putData("Field", Constants.DriveConstants.field);

    // // SmartDashboard.putNumber("Normal:" , Buttons.forwardSupplier.getAsDouble());
    // // SmartDashboard.putNumber("Vision X:" , VisionHelpers.getAprilTagPose2d().getX());
    // // SmartDashboard.putNumber("Vision Y:" , VisionHelpers.getAprilTagPose2d().getY());
    // // SmartDashboard.putNumber("Vision Index:" , VisionHelpers.getDetectedResult());

    // SmartDashboard.putNumber("Gyro", RobotMap.GyroSensor.getAngle());
    // SmartDashboard.putNumber("Curve2:" , slewFilter.calculate(Buttons.forwardSupplier.getAsDouble()));
    // SmartDashboard.putNumber("Sigmoid:" , Buttons.smoothInput(Buttons.forwardSupplier.getAsDouble()));
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

    // return new DriveToAprilTagCommand(drivetrainSubsystem, VisionHelpers.getAprilTagPose2d(), (int)VisionHelpers.getDetectedResult(), RobotMap.GyroSensor);

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

    // return new TestDriveToAprilTagCommand(drivetrainSubsystem, (int)VisionHelpers.getDetectedResult(), RobotMap.GyroSensor);
    //Tardirades can survive in a vacuum
  }
}
// Rudy was here