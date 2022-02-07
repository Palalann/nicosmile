// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


//THIS CODE IS FOR SIMPLE DRIVETRAIN 
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import edu.wpi.first.math.geometry.Translation2d;

import static frc.robot.Constants.SIMDB.*;
import static frc.robot.Constants.Joystick.*;




public class Simpledb extends SubsystemBase {
  /** Creates a new DriveTrain. */
  public WPI_TalonSRX rightMaster = new WPI_TalonSRX(RMMOTOR_ID);
  public WPI_TalonSRX rightFollow = new WPI_TalonSRX(RFMOTOR_ID);
  public WPI_TalonSRX leftMaster = new WPI_TalonSRX(LMMOTOR_ID);
  public WPI_TalonSRX leftFollow = new WPI_TalonSRX(LFMOTOR_ID);

  
  


  public Simpledb() {
    rightFollow.follow(rightMaster);
    leftFollow.follow(leftMaster);
    leftFollow.setInverted(true);
    leftMaster.setInverted(true);

    rightFollow.setNeutralMode(NeutralMode.Brake);
    rightMaster.setNeutralMode(NeutralMode.Brake);
    leftMaster.setNeutralMode(NeutralMode.Brake);
    leftMaster.setNeutralMode(NeutralMode.Brake);
  }
    
    public void drive(double leftDrive, double rightDrive){
      leftMaster.set(leftDrive);
      rightMaster.set(rightDrive);

    }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double l2 = RobotContainer.controller.getRawButton(L2)? 0.8: 0.4;
    
    if(RobotContainer.controller.getRawAxis(1) < -0.5){
      drive(l2 * 0.8, l2 * 0.8);
    }
    else if (RobotContainer.controller.getRawAxis(1) > 0.5){
      drive(l2 *-0.8, l2 * -0.8);
    }
    else if (RobotContainer.controller.getRawAxis(0) < -0.5){
      drive(l2 * 0.4, l2 * 0.8);
    }    
    else if (RobotContainer.controller.getRawAxis(0) > 0.5){
      drive(l2 * 0.8, l2 * 0.4);
    }


    else if (RobotContainer.controller.getRawAxis(1) < -0.5 && RobotContainer.controller.getRawAxis(0) < -0.5){
      drive(l2 * 0.4, l2 * 0.75);
    }
    else if (RobotContainer.controller.getRawAxis(1) < -0.5 && RobotContainer.controller.getRawAxis(0) > 0.5){
      drive(l2 * 0.75, l2 * 0.4);
    } 
    else if (RobotContainer.controller.getRawAxis(1) > 0.5 && RobotContainer.controller.getRawAxis(0) < -0.5){
      drive(l2 * -0.4, l2 * -0.75);
    } 
    else if (RobotContainer.controller.getRawAxis(1) < -0.5 && RobotContainer.controller.getRawAxis(0) > 0.5){
      drive(l2 * -0.75, l2 * -0.4);
    } 

    else {
      drive(l2 * 0.4, l2 * 0.4);
    }
  }
}