// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import static frc.robot.Constants.OMNIH.*;
import frc.robot.RobotContainer;
import static frc.robot.Constants.Joystick.*;

public class OmniHdb extends SubsystemBase {
  /** Creates a new OmniHdb. */
  public static WPI_TalonSRX vertmaster1 = new WPI_TalonSRX(VERTMASTER_MOTOR1_ID);
  public static WPI_TalonSRX vertmaster2 = new WPI_TalonSRX(VERTMASTER_MOTOR2_ID);
  public static WPI_TalonSRX horimaster = new WPI_TalonSRX(HORIMASTER_MOTOR1_ID);

  public OmniHdb() {
    vertmaster1.setNeutralMode(NeutralMode.Brake);
    vertmaster2.setNeutralMode(NeutralMode.Brake);
    horimaster.setNeutralMode(NeutralMode.Brake);
  }
  public static void drive(double vert1V, double vert2V, double horiV){
    vertmaster1.set(vert1V);
    vertmaster2.set(vert2V);
    horimaster.set(horiV);
    
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double l2 = RobotContainer.controller.getRawButton(L2)? 0.8 : 0.4;

    //Go straight forward
    if(RobotContainer.controller.getRawAxis(1) < -0.5 ){
      drive(l2 * 0.8, l2 * 0.8 , l2 * 0);
    }
    //Go straight backward
    else if(RobotContainer.controller.getRawAxis(1) > 0.5){
      drive(l2 * -0.8, l2 * -0.8 , l2 * -0);
    }

    //Turn left forward
     else if(RobotContainer.controller.getRawAxis(0) < -0.5) {
      drive(l2 * 0.8, l2 * 0.4 , l2 * 0);
    }
    //Turn right forward
    else if(RobotContainer.controller.getRawAxis(0) > 0.5) {
      drive(l2 * 0.4, l2 * 0.8 , l2 * 0);
    }
    //Turn left backward
    else if(RobotContainer.controller.getRawAxis(0) < -0.5) {
      drive(l2 * -0.8, l2 *- 0.4 , l2 * 0);
    }
    //Turn right
    else if(RobotContainer.controller.getRawAxis(0) > 0.5) {
      drive(l2 * -0.4, l2 * -0.8 , l2 * 0);
    }

    //Straight right side
    else if(RobotContainer.controller.getRawAxis(2) > 0.5){
      drive(l2 * 0, l2 * 0, l2 * 0.8);
    }
    //Straight left side
    else if(RobotContainer.controller.getRawAxis(2) > 0.5){
      drive(l2 * 0, l2 * 0, l2 * -0.8);
    }


    

  }
}
