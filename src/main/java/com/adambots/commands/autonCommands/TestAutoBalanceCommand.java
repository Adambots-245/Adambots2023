// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.commands.autonCommands;

import com.adambots.Constants.GrabbyConstants;
import com.adambots.sensors.Gyro;
import com.adambots.subsystems.DrivetrainSubsystem;
import com.adambots.subsystems.GrabbyLifterSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class TestAutoBalanceCommand extends CommandBase {
  private DrivetrainSubsystem m_drivetrainSubsystem;
  private Gyro m_gyro;
  private int state;
  private int inc;
  private int revInc;
  // private int finalInc;
  private int firstInc;
  private int debounce;
  private GrabbyLifterSubsystem grabbyLifterSubsystem;

  /** Creates a new AutoBalanceCommand. */
  public TestAutoBalanceCommand(DrivetrainSubsystem drivetrainSubsystem, Gyro gyro, GrabbyLifterSubsystem grabbyLifterSubsystem) {
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
    inc = 0;
    revInc = 0;
    // finalInc = 0;
    firstInc = 0;
    debounce = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double pitchAngleDegrees = m_gyro.getPitch();
    inc++;

    if (state == 0) { //Initial getting onto drivestation at higher speed
      m_drivetrainSubsystem.drive(1, 0, 0, true);
      if (Math.abs(pitchAngleDegrees) > 11) {
        if (firstInc == 0) {
          grabbyLifterSubsystem.changeTarget(GrabbyConstants.balancingState.getArmLiftTarget());
        }
        firstInc++;
      }
      if (firstInc > 40) { //Drive for 30 ticks after front wheels get up to get back wheels up
        state = 1;
      }
    }

    if (state == 1) { //Drive at slower speed until platform tips
      m_drivetrainSubsystem.drive(0.2, 0, 0, true);
      if (Math.abs(m_gyro.getPitch()) < 5) {
        state = 2;
      }
    }

    if (state == 2) { //Drive in reverse for a set time
      m_drivetrainSubsystem.drive(-0.1, 0, 0, true);
      revInc++;
      if (revInc > 35) { //TUNE THIS FOR REVERSE TIME
        state = 3;
      }
    }

    // if (state == 3) {
    //   // if (finalInc > 0) {
    //     if (m_gyro.getPitch() < 7) {
    //       m_drivetrainSubsystem.drive(-0.075, 0, 0, true);
    //       debounce = Math.max(debounce-1, 0);
    //     }
    //     else if (m_gyro.getPitch() > 7) {
    //       m_drivetrainSubsystem.drive(0.075, 0, 0, true);
    //       debounce = Math.max(debounce-1, 0);
    //     }
    //     else {
    //       m_drivetrainSubsystem.stop();
    //       debounce++;
    //       if (debounce > 20) {
    //         state = 4;
    //       }
    //     }
      // } 
    //   finalInc++;
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrainSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return state == 4;
    return state == 3;
  }
}
