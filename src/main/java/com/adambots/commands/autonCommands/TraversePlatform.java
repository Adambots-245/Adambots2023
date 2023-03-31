// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.commands.autonCommands;

import com.adambots.Constants.AutoConstants;
import com.adambots.sensors.Gyro;
import com.adambots.subsystems.DrivetrainSubsystem;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TraversePlatform extends CommandBase {
  private DrivetrainSubsystem m_drivetrainSubsystem;
  private Gyro m_gyro;
  private int state;
  private int inc1;
  private int inc2;
  private PIDController thetaController;

  /** Creates a new AutoBalanceCommand. */
  public TraversePlatform(DrivetrainSubsystem drivetrainSubsystem, Gyro gyro) {
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(drivetrainSubsystem);
    m_drivetrainSubsystem = drivetrainSubsystem;
    m_gyro = gyro;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    state = 0;
    inc1 = 0;
    inc2 = 0;

    thetaController = new PIDController(AutoConstants.kPThetaController, 0, AutoConstants.kDThetaController);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    // thetaController.setSetpoint(Math.PI+Math.toRadians(25));
    thetaController.setSetpoint(Math.PI);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double pitchAngleDegrees = m_gyro.getPitch() + m_gyro.getRoll();

    if (state == 0) { //Initial getting onto drivestation at higher speed
      m_drivetrainSubsystem.drive(1, 0, 0, true);
      if (Math.abs(pitchAngleDegrees) > 5) {
        inc1++;
      }
      if (inc1 > 60) { //Drive for 30 ticks after front wheels get up to get back wheels up
        state = 1;
      }
    }

    if (state == 1) { //Drive at slower speed until platform tips
      double rot = thetaController.calculate(Math.toRadians(m_gyro.getYaw()));
      m_drivetrainSubsystem.drive(0.5, 0, -rot, true);
      if (Math.abs(pitchAngleDegrees) < 2) {
        inc2++;
      } else {
        inc2 = 0;
      }
      // if (inc2 > 15) { //Drive for 30 ticks after front wheels get up to get back wheels up
      if (inc2 > 20) { //Drive for 30 ticks after front wheels get up to get back wheels up
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
    return state == 2 && Math.abs(m_gyro.getPitch()) < 2;
  }
}
