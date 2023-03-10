// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.commands.autonCommands;

import com.adambots.sensors.Gyro;
import com.adambots.subsystems.DrivetrainSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class TestAutoBalanceCommand extends CommandBase {
  private DrivetrainSubsystem m_drivetrainSubsystem;
  private Gyro m_gyro;
  private int state;
  private int inc;
  private int revInc;
  private int finalInc;
  private int firstInc;
  private int debounce;

  /** Creates a new AutoBalanceCommand. */
  public TestAutoBalanceCommand(DrivetrainSubsystem drivetrainSubsystem, Gyro gyro) {
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(drivetrainSubsystem);
    m_drivetrainSubsystem = drivetrainSubsystem;
    m_gyro = gyro;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    state = 0;
    inc = 0;
    revInc = 0;
    finalInc = 0;
    firstInc = 0;
    debounce = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double pitchAngleDegrees = m_gyro.getPitch();
    inc++;

    if (state == 0) { //Initial getting onto drivestation at higher speed
      m_drivetrainSubsystem.drive(-0.5, 0, 0, false);
      if (Math.abs(pitchAngleDegrees) > 11) {
        firstInc++;
      }
      if (firstInc > 30) { //Drive for 30 ticks after front wheels get up to get back wheels up
        state = 1;
      }
    }

    if (state == 1) { //Drive at slower speed until platform tips
      m_drivetrainSubsystem.drive(-0.125, 0, 0, false);
      if (Math.abs(m_gyro.getPitch()) < 5) {
        state = 2;
      }
    }

    if (state == 2) { //Drive in reverse for a set time
      m_drivetrainSubsystem.drive(0.125, 0, 0, false);
      revInc++;
      if (revInc > 80) { //TUNE THIS FOR REVERSE TIME
        state = 3;
      }
    }

    // if (state == 3) {
    //   if (finalInc > 0) {
    //     if (m_gyro.getPitch() < 7) {
    //       m_drivetrainSubsystem.drive(0.075, 0, 0, false);
    //     }
    //     else if (m_gyro.getPitch() > 7) {
    //       m_drivetrainSubsystem.drive(-0.075, 0, 0, false);
    //     }
    //     else {
    //       debounce++;
    //       if (debounce > 20) {
    //         state = 4;
    //       }
    //       else {
    //         debounce = Math.max(debounce-1, 0);
    //       }
    //     }
    //   } else {
    //     m_drivetrainSubsystem.drive(0, 0, 0, false);
    //   }
    //   finalInc++;
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrainSubsystem.stop();
    new HockeyStopCommand(m_drivetrainSubsystem).schedule();;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return inc > 600 || state == 4;
    return state == 3;
  }
}
