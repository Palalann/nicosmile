// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


//THIS CODE IS FOR OMNI X-DRIVETRAIN WITH 4 OMNI WHEEL, EACH ONE AT A CONER
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import static frc.robot.Constants.OMNIX.*;
import static frc.robot.Constants.Joystick.*;

public class OmniXdb extends SubsystemBase {
  /** Creates a new Omnidb. */
  public static WPI_TalonSRX left_forward = new WPI_TalonSRX(LFMOTOR);
  public static WPI_TalonSRX right_forward = new WPI_TalonSRX(RFMOTOR);
  public static WPI_TalonSRX left_backward = new WPI_TalonSRX(LBMOTOR);
  public static WPI_TalonSRX right_backward = new WPI_TalonSRX(RBMOTOR);
  public OmniXdb() {
    left_backward.setNeutralMode(NeutralMode.Brake);
    left_forward.setNeutralMode(NeutralMode.Brake);
    right_backward.setNeutralMode(NeutralMode.Brake);
    right_forward.setNeutralMode(NeutralMode.Brake);
  }

  public void drive(double lfV, double rfV, double lbV, double rbV){
    left_forward.set(lfV);
    right_forward.set(rfV);
    left_backward.set(lbV);
    right_backward.set(rbV);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double l2 = RobotContainer.controller.getRawButton(L2)? 0.8 : 0.4;

    //Go straight forward
    if(RobotContainer.controller.getRawAxis(1) < -0.5 ){
      drive(l2 * 0.8, l2 * 0.8 , l2 * 0.8, l2 * 0.8);
    }
    //Go straight backward
    else if(RobotContainer.controller.getRawAxis(1) > 0.5){
      drive(l2 * -0.8, l2 * -0.8 , l2 * -0.8, l2 * -0.8);
    }
    //Go straight leftside
    else if(RobotContainer.controller.getRawAxis(0) < -0.5) {
      drive(l2 * -0.8, l2 * 0.8 , l2 * -0.8, l2 * 0.8);
    }
    //Go straigt rightside
    else if(RobotContainer.controller.getRawAxis(0) > 0.5) {
      drive(l2 * -0.8, l2 * 0.8 , l2 * -0.8, l2 * 0.8);
    }


    //Go cross right forward
    else if((RobotContainer.controller.getRawAxis(1) < -0.3) && (RobotContainer.controller.getRawAxis(0) < -0.3)){
      drive(l2 * 0.8, l2 * 0 , l2 * 0.8, l2 * 0);
    }
    //Go cross left forward
    else if((RobotContainer.controller.getRawAxis(1) < -0.3) && (RobotContainer.controller.getRawAxis(0) > 0.3)){
      drive(l2 * 0, l2 * 0.8 , l2 * 0, l2 * 0.8);
    }
     //Go cross left backward
     else if((RobotContainer.controller.getRawAxis(1) > 0.3) && (RobotContainer.controller.getRawAxis(0) < -0.3)){
      drive(l2 * -0.8, l2 * 0 , l2 * -0.8, l2 * 0);
    }
     //Go cross right backward
     else if((RobotContainer.controller.getRawAxis(1) > 0.3) && (RobotContainer.controller.getRawAxis(0) > 0.3)){
      drive(l2 * 0, l2 * -0.8 , l2 * 0, l2 * -0.8);
    }


    //Turn left around
    if (RobotContainer.controller.getRawAxis(2) < -0.3){
      drive(l2 * -0.8, l2 * 0.8 , l2 * -0.8, l2 * 0.8);
    }
    //Turn right around
    if(RobotContainer.controller.getRawAxis(2) > 0.3) {
      drive(l2 * -0.8, l2 * 0.8 , l2 * -0.8, l2 * 0.8);
    }
  }
}
