// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Simpledb;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveStraight extends CommandBase {
  /** Creates a new Drive. */
  public Simpledb m_drivetrain;
  public double leftSpeed;
  public double rightSpeed;
  public DriveStraight(Simpledb drivetrain, double lv, double rv) {
    m_drivetrain = drivetrain;
    leftSpeed = lv;
    rightSpeed = rv;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrain.drive(leftSpeed, rightSpeed);
    SmartDashboard.putNumber("Left Speed", leftSpeed);
    SmartDashboard.putNumber("Right speed", rightSpeed);
    SmartDashboard.putBoolean("Go", true);
    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.drive(0, 0);
    SmartDashboard.putBoolean("Go", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
