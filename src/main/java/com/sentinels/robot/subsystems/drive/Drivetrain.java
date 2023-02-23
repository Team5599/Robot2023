/***************************************************************
                The Sentinels - FRC Team 5599
        Benjamin N. Cardozo High School Robotics Team

    This work is licensed under the terms of the MIT license.
    Copyright (c) 2023 The Sentinels. All rights reserved.
***************************************************************/

package com.sentinels.robot.subsystems.drive;

import com.sentinels.robot.constants.Motors;
import com.sentinels.robot.constants.Ports;
import com.sentinels.robot.constants.Settings;
import com.sentinels.robot.util.RoboRIO;
import com.sentinels.robot.subsystems.odometry.IMU;

import java.util.List;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.simulation.ADIS16470_IMUSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;

/**
 * Code to allow the robot to move.
 * 
 * <p>Tank Drive contains:
 * <p>- 2x NEO Motors w/ encoders on the LEFT side (front and back)
 * <p>- 2x NEO Motors w/ encoders on the RIGHT side (front and back)
 * <p>- ADIS16470 IMU connected to roboRIO SPI port
 * 
 * @author Ahmed Osman, Karamat Hasan
 */
public class Drivetrain extends SubsystemBase {

  // LEFT side motors
  private final CANSparkMax motorFL = new CANSparkMax(Ports.Drivetrain.FRONTLEFT, MotorType.kBrushless);
  private final CANSparkMax motorBL = new CANSparkMax(Ports.Drivetrain.BACKLEFT, MotorType.kBrushless);
  
  // RIGHT side motors
  private final CANSparkMax motorFR = new CANSparkMax(Ports.Drivetrain.FRONTRIGHT, MotorType.kBrushless);
  private final CANSparkMax motorBR = new CANSparkMax(Ports.Drivetrain.BACKRIGHT, MotorType.kBrushless);

  private final MotorControllerGroup leftMotors = new MotorControllerGroup(motorFL, motorBL);
  private final MotorControllerGroup rightMotors = new MotorControllerGroup(motorFR, motorBR);

  private final DifferentialDrive drivetrain = new DifferentialDrive(leftMotors, rightMotors);

  // Built-in NEO hall sensor encoders
  private final RelativeEncoder encoderFL = motorFL.getEncoder();
  private final RelativeEncoder encoderBL = motorBL.getEncoder();
  private final RelativeEncoder encoderFR = motorFR.getEncoder();
  private final RelativeEncoder encoderBR = motorBR.getEncoder();

  private final ADIS16470_IMU imu = new ADIS16470_IMU();

  private final DifferentialDrivePoseEstimator odometry = new DifferentialDrivePoseEstimator(
    Settings.Drivetrain.KINEMATICS, getHeading(), getLeftPosition(), getRightPosition(), new Pose2d()
  );

  // SIMULATION

  private final Encoder sEncoderL = new Encoder(1, 2);
  private final Encoder sEncoderR = new Encoder(5, 6);

  private final EncoderSim simEncoderL = new EncoderSim(sEncoderL);
  private final EncoderSim simEncoderR = new EncoderSim(sEncoderR);

  private final ADIS16470_IMUSim imuSim = new ADIS16470_IMUSim(imu);

  private final Trajectory trajectory;
  
  private Field2d field = new Field2d();

  public Drivetrain() {
    // Resetting motor settings to default factory settings
    motorFL.restoreFactoryDefaults();
    motorBL.restoreFactoryDefaults();
    motorFR.restoreFactoryDefaults();
    motorBR.restoreFactoryDefaults();

    // Zeroing encoder positions
    resetEncoders();

    // Invert the one of the sides so that they rotate synonymously in one direction
    leftMotors.setInverted(true);
    
    simEncoderL.setDistancePerPulse(2 * Math.PI * (Settings.Drivetrain.kWheelDiameter / 2) / 42);
    simEncoderR.setDistancePerPulse(2 * Math.PI * (Settings.Drivetrain.kWheelDiameter / 2) / 42);

    // Create the trajectory to follow in autonomous. It is best to initialize
    // trajectories here to avoid wasting time in autonomous.
    trajectory =
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            new Pose2d(3, 0, Rotation2d.fromDegrees(0)),
            new TrajectoryConfig(Units.feetToMeters(3.0), Units.feetToMeters(3.0)));

    // Push the trajectory to Field2d.
    field.getObject("traj").setTrajectory(trajectory);

    SmartDashboard.putData("Field", field);
  }

  /**
   * Drive the robot!
   * @param leftSpeed - The speed at which the left side motors should be.
   * @param rightSpeed - The speed at which the right side motors should be.
   */
  public void tankDrive(double leftSpeed, double rightSpeed) {
    drivetrain.tankDrive(leftSpeed, rightSpeed);
  }
  public void arcadeDrive(double xSpeed, double rotation) {
    drivetrain.arcadeDrive(xSpeed, rotation);
  }
  public void voltageDrive(double leftVoltage, double rightVoltage){
    leftMotors.setVoltage(leftVoltage);
    rightMotors.setVoltage(rightVoltage);
    drivetrain.feed();
  }

  // Stop the motors from moving
  public void driveStop() {
    drivetrain.stopMotor();
  }

  /**
   * Manually change the speed of both MotorControllerGroups {@code leftMotors} and {@code rightMotors}.
   * @param leftSpeed - The speed at which the left side motors should be.
   * @param rightSpeed - The speed at which the right side motors should be.
   */
  public void setSpeed(double leftSpeed, double rightSpeed) {
    leftMotors.set(leftSpeed);
    rightMotors.set(rightSpeed);
  }

  // POSITION FUNCTIONS

  public double getLeftPosition() {
    return (encoderFL.getPosition() + encoderBL.getPosition() / 2.0);
  }
  public double getRightPosition() {
    return (encoderFR.getPosition() + encoderBR.getPosition() / 2.0);
  }

  // VELOCITY FUNCTIONS

  public double getLeftVelocity() {
    return (encoderFL.getVelocity() + encoderBL.getVelocity() / 2.0);
  }
  public double getRightVelocity() {
    // negate so both velocities are positive
    return -(encoderFR.getVelocity() + encoderBR.getVelocity() / 2.0);
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds(){
    double AvgLeftVel = 0.254 * (getLeftVelocity() * 2 * Math.PI * (Settings.Drivetrain.kWheelDiameter / 2)) / 60;
    double AvgRightVel = 0.254 * (getRightVelocity() * 2 * Math.PI * (Settings.Drivetrain.kWheelDiameter / 2)) / 60;
    return new DifferentialDriveWheelSpeeds(AvgLeftVel, AvgRightVel);// in m/s
  }

  // VOLTAGE FUNCTIONS

  public double getLeftVoltage() {
    return (leftMotors.get() * RoboRIO.getBatteryVoltage());
  }
  public double getRightVoltage() {
    return (rightMotors.get() * RoboRIO.getBatteryVoltage());
  }
  public void setMaxOutput(double maxOutput){
    drivetrain.setMaxOutput(maxOutput);
  }

  // RESET ENCODERS
  
  public void resetEncoders(){
    encoderFL.setPosition(0);
    encoderBL.setPosition(0);
    encoderFR.setPosition(0);
    encoderBR.setPosition(0);
  }

  // IMU FUNCTIONS + TRAJECTORY 

  public Rotation2d getHeading() {
    return new Rotation2d(-imu.getAngle());
  }
  public double getTurnRate() {
    return -imu.getRate();
  }
  public void zeroIMU() {
    imu.reset();
  }
  public Pose2d getPose() {
    return odometry.getEstimatedPosition();
  }

  // see line 102 of docs

  // public void resetOdometry(){
  //   resetEncoders();
  // }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Drivetrain Left Motors Voltage (V)", getLeftVoltage());
    SmartDashboard.putNumber("Drivetrain Right Motors Voltage (V)", getRightVoltage());

    SmartDashboard.putNumber("Drivetrain Left Motors Position (Rotations)", getLeftPosition());
    SmartDashboard.putNumber("Drivetrain Right Motors Position (Rotations)", getRightPosition());

    SmartDashboard.putNumber("Drivetrain Left Motors Velocity (RPM)", getLeftVelocity());
    SmartDashboard.putNumber("Drivetrain Right Motors Velocity (RPM)", getRightVelocity());

    // This will get the simulated sensor readings that we set
    // in the previous article while in simulation, but will use
    // real values on the robot itself.
    odometry.update(getHeading(), simEncoderL.getDistance(), simEncoderR.getDistance());
    field.setRobotPose(odometry.getEstimatedPosition());
  }

  @Override
  public void simulationPeriodic() {
    // Set the inputs to the system. Note that we need to convert
    // the [-1, 1] PWM signal to voltage by multiplying it by the
    // robot controller voltage.
    SimDrivetrain.simDrivetrain.setInputs(leftMotors.get() * RoboRIO.getInputVoltage(), rightMotors.get() * RoboRIO.getInputVoltage());
  
    // Advance the model by 20 ms. Note that if you are running this
    // subsystem in a separate thread or have changed the nominal timestep
    // of TimedRobot, this value needs to match it.
    SimDrivetrain.simDrivetrain.update(0.02);
  
    // Update all of our sensors.
    simEncoderL.setDistance(SimDrivetrain.simDrivetrain.getLeftPositionMeters());
    simEncoderL.setRate(SimDrivetrain.simDrivetrain.getLeftVelocityMetersPerSecond());
    simEncoderR.setDistance(SimDrivetrain.simDrivetrain.getRightPositionMeters());
    simEncoderR.setRate(SimDrivetrain.simDrivetrain.getRightVelocityMetersPerSecond());
    imuSim.setGyroAngleX(-SimDrivetrain.simDrivetrain.getHeading().getDegrees());
  }
}
