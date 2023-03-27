// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.commands.autonCommands;

import com.adambots.Constants.GrabbyConstants;
import com.adambots.sensors.Gyro;
import com.adambots.subsystems.DrivetrainSubsystem;
import com.adambots.subsystems.GrabbyLifterSubsystem;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoBalanceCommand extends CommandBase {
  private DrivetrainSubsystem m_drivetrainSubsystem;
  private Gyro m_gyro;
  private int state;
  private int firstInc;
  private int balInc;
  private PIDController pid;
  private GrabbyLifterSubsystem grabbyLifterSubsystem;

  /** Creates a new AutoBalanceCommand. */
  public AutoBalanceCommand(DrivetrainSubsystem drivetrainSubsystem, Gyro gyro, GrabbyLifterSubsystem grabbyLifterSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(drivetrainSubsystem);
    m_drivetrainSubsystem = drivetrainSubsystem;
    m_gyro = gyro;
    this.grabbyLifterSubsystem = grabbyLifterSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    state = 0;
    firstInc = 0;
    balInc = 0;

    pid = new PIDController(0.01, 0, 0.0017);
    pid.setSetpoint(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double pitchAngleDegrees = m_gyro.getPitch();

    if (state == 0) { //Initial getting onto drivestation at higher speed
      m_drivetrainSubsystem.drive(-0.7, 0, 0, true);
      if (Math.abs(pitchAngleDegrees) > 3) {
        if (firstInc == 0) {
          grabbyLifterSubsystem.changeTarget(GrabbyConstants.balancingState.getArmLiftTarget());
        }
        firstInc++;
      }
      if (firstInc > 25) { //Drive for 30 ticks after front wheels get up to get back wheels up
        state = 1;
      }
    }

    if (state == 1) { //Drive at slower speed until platform tips
      double speed = pid.calculate(pitchAngleDegrees);
      m_drivetrainSubsystem.drive(speed, 0, 0, true);

      if (Math.abs(pitchAngleDegrees) < 3) {
        balInc++;
      }
      if (balInc > 50) { //Drive for 30 ticks after front wheels get up to get back wheels up
        state = 2;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrainSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return state == 2;
  }
}
