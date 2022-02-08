// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


//THIS CODE IS FOR SIMPLE DRIVETRAIN, BUT USE WITH TRAJECTORY
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import java.lang.Math;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.wpilibj2.command.RamseteCommand;
import static frc.robot.Constants.RAMSETE.*;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;

import edu.wpi.first.math.kinematics.ChassisSpeeds;


import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import static frc.robot.Constants.DIFFDRIVE.*;
import static frc.robot.Constants.MEASUREMENT.*;
import static frc.robot.Constants.Joystick.*;
import static frc.robot.Constants.PiAiDi.*;

import edu.wpi.first.math.filter.SlewRateLimiter;

public class Differentialdb extends SubsystemBase {
  /** Creates a new Drivetrain. */
  //Right Group
  public static WPI_TalonSRX rightMaster = new WPI_TalonSRX(RMMOTOR);
  public static WPI_TalonSRX rightFollow = new WPI_TalonSRX(RFMOTOR);
  public final MotorControllerGroup rightGroup = new MotorControllerGroup(rightMaster, rightFollow);
  //Left Group
  public static WPI_TalonSRX leftMaster = new WPI_TalonSRX(LMMOTOR);
  public static WPI_TalonSRX leftFollow = new WPI_TalonSRX(LFMOTOR);
  public final MotorControllerGroup leftGroup = new MotorControllerGroup(leftMaster, leftFollow);
  //Encoders
  public static final Encoder leftEncoder = new Encoder(0, 1);
  public static final Encoder rightEncoder = new Encoder(2, 3);
  //Gyro
  public static final AnalogGyro gyro = new AnalogGyro(0);
  //Piaidi
  public static final PIDController leftPID = new PIDController(kPL, kIL, kDL);
  public static final PIDController rightPID = new PIDController(kPR, kIR, kDR);
  //Kinetics
  public final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(TRACKWIDTH));
  //Odometry
  //Get the initial place of the bot
  private final DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(gyro.getRotation2d()); 
  //Diferential
  private final DifferentialDrive drive = new DifferentialDrive(leftGroup, rightGroup);
 
  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(1, 3);
 

  public final Joystick controller = new Joystick(1);

  private final SlewRateLimiter m_speedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  
  

  public Differentialdb() {
    //reset gyro
    gyro.reset();

    rightFollow.follow(rightMaster);
    leftFollow.follow(leftMaster);
    /*set inverted one side of drivetrain so that positive voltage
    means that both side moving forward */
    leftGroup.setInverted(true);

    //Set distance per pulse for each encoder
    //Simply use distance travelled per one rotation of the wheel/ encoder resolution
    leftEncoder.setDistancePerPulse(2 * Math.PI * WHEEL_RADIUS / ENCODER_RESOLUTION);
    rightEncoder.setDistancePerPulse(2 * Math.PI * WHEEL_RADIUS / ENCODER_RESOLUTION);
    //Reset the encoder
    leftEncoder.reset();
    rightEncoder.reset();    

  }
  //Set the desire speed
 public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    final double leftFeedforward = feedforward.calculate(speeds.leftMetersPerSecond);
    final double rightFeedforward = feedforward.calculate(speeds.rightMetersPerSecond);

    final double leftOutput = leftPID.calculate(leftEncoder.getRate(), speeds.leftMetersPerSecond);
    final double rightOutput = rightPID.calculate(rightEncoder.getRate(), speeds.rightMetersPerSecond);
    leftGroup.setVoltage(leftOutput + leftFeedforward);
    rightGroup.setVoltage(rightOutput + rightFeedforward);
  }
   
  public void drive(double xSpeed, double rot) { 
    var wheelSpeeds = kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, rot));
    setSpeeds(wheelSpeeds);    
  }


  


  @Override
  public void periodic() {
      odometry.update(gyro.getRotation2d(), 
                      leftEncoder.getDistance(), rightEncoder.getDistance());
      // Get the x speed. We are inverting this because controllers return
    // negative values when we push forward.
    final var xSpeed = -m_speedLimiter.calculate(controller.getRawAxis(1)) * kMaxSpeed;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). controllers return positive values when you pull to
    // the right by default.
    final var rot = -m_rotLimiter.calculate(controller.getRawAxis(0)) * kMaxAngularSpeed;

    drive(xSpeed, rot);
  }

  //Return the current estimate pose of the robot
  public Pose2d getPose(){
    return odometry.getPoseMeters();
  }

  //Return the current wheel speed
  public DifferentialDriveWheelSpeeds getWheelSpeeds(){
    return new DifferentialDriveWheelSpeeds(leftEncoder.getRate(), rightEncoder.getRate());
  }

  //Resets the odometry to specified pose
  public void resetOdometry(Pose2d pose) {
    rightEncoder.reset();
    leftEncoder.reset();
    
    odometry.resetPosition(pose, gyro.getRotation2d());
  }

  //Use arcade control to drive robot
  public void arcadeDrive(double forwardMovement, double rotation) {
    drive.arcadeDrive(forwardMovement, rotation);
  }

  //Control the left and right side of the drive with voltage
  public void tankDriveVolts(double leftVolts, double rightVolt){
    leftGroup.setVoltage(leftVolts);
    rightGroup.setVoltage(rightVolt);
    drive.feed();
  }

  //Reset the drive encoder to read the current position of theta
  public void resetEncoders(){
    leftEncoder.reset();
    rightEncoder.reset();
  }

  //Input : Average distance of 2 encoders, Output : Average of the two encoder reading
  public double getAverageEncoderDistance() {
    return ((leftEncoder.getDistance() + rightEncoder.getDistance())/2);
  }

  //Gét the lèt drive encoder
  public Encoder getLeftEncoder() {
    return leftEncoder;
  }

  //Gets the right drive encoder
  public Encoder getRightEncoder() {
    return rightEncoder;
  }

  //Set the maximum output. Useful to scaling to drive more slowly
  public void setMaximumOutput(double maxOutput){
    drive.setMaxOutput(maxOutput);
  }

  //Zero the headung of the robot
  public double getHeading(){
    return gyro.getRotation2d().getDegrees();
  }

  //Output : the turn rate of the robot
  public double getTurnRate() {
    return -gyro.getRate();
  }
  

  }

  
/* For Encoder :
1. Specified encoder ports 
2. Specified encoder distance per pulse. Note that pulse in WPI = 1/2 pulse in Sysid
3. Access Encoder by method : getWheelSpeed(). Note that returned velo is in meter

  For Gyro
1. Create a new Gyro
2. Access Gyro by method

  For Odometry
  1. Create a member instance of DifferentialDriveOdometry class
  2. Update the Odometry
  3. Access Odometry

  For VoltageBased Drive 
*/
