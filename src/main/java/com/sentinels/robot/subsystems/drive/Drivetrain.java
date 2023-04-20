/***************************************************************
                The Sentinels - FRC Team 5599
        Benjamin N. Cardozo High School Robotics Team

    This work is licensed under the terms of the MIT license.
    Copyright (c) 2023 The Sentinels. All rights reserved.
***************************************************************/

package com.sentinels.robot.subsystems.drive;

import com.sentinels.robot.constants.Ports;
import com.sentinels.robot.constants.Settings;
import com.sentinels.robot.util.RoboRIO;
import com.sentinels.robot.util.SparkMAXsim;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.simulation.ADIS16470_IMUSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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
  private final CANSparkMax motorFL = new SparkMAXsim(Ports.Drivetrain.FRONTLEFT, MotorType.kBrushless);
  private final CANSparkMax motorBL = new SparkMAXsim(Ports.Drivetrain.BACKLEFT, MotorType.kBrushless);
  
  // RIGHT side motors
  private final CANSparkMax motorFR = new SparkMAXsim(Ports.Drivetrain.FRONTRIGHT, MotorType.kBrushless);
  private final CANSparkMax motorBR = new SparkMAXsim(Ports.Drivetrain.BACKRIGHT, MotorType.kBrushless);

  private final MotorControllerGroup leftMotors = new MotorControllerGroup(motorFL, motorBL);
  private final MotorControllerGroup rightMotors = new MotorControllerGroup(motorFR, motorBR);

  private final DifferentialDrive drivetrain = new DifferentialDrive(leftMotors, rightMotors);

  // Built-in NEO hall sensor encoders
  private final RelativeEncoder encoderFL = motorFL.getEncoder();
  private final RelativeEncoder encoderBL = motorBL.getEncoder();
  private final RelativeEncoder encoderFR = motorFR.getEncoder();
  private final RelativeEncoder encoderBR = motorBR.getEncoder();

  private final ADIS16470_IMU imu = new ADIS16470_IMU();

  // SIMULATION

  private final Encoder sEncoderL = new Encoder(1, 2);
  private final Encoder sEncoderR = new Encoder(5, 6);

  private final EncoderSim simEncoderL = new EncoderSim(sEncoderL);
  private final EncoderSim simEncoderR = new EncoderSim(sEncoderR);

  private final ADIS16470_IMUSim imuSim = new ADIS16470_IMUSim(imu);
  
  private Field2d field = new Field2d();

  private DifferentialDrivetrainSim SimDrivetrain;
  
  private final DifferentialDrivePoseEstimator odometry = new DifferentialDrivePoseEstimator(
    Settings.Drivetrain.kDriveKinematics, getHeading(), getLeftPosition(), getRightPosition(), new Pose2d()
  );

  public Drivetrain() {
    motorFL.restoreFactoryDefaults();
    motorBL.restoreFactoryDefaults();
    motorFR.restoreFactoryDefaults();
    motorBR.restoreFactoryDefaults();

    // Invert the one of the sides so that they rotate synonymously in one direction
    leftMotors.setInverted(true);

    // Set a max output to avoid damage
    setMaxOutput(0.95);
    
    // Zeroing and calibrating IMU, zeroing encoder positions
    zeroIMU();
    resetEncoders();

    // Set velocity and position factors
    encoderFL.setPositionConversionFactor(2 * Math.PI * Units.inchesToMeters(3));
    encoderBL.setPositionConversionFactor(2 * Math.PI * Units.inchesToMeters(3));
    encoderFR.setPositionConversionFactor(2 * Math.PI * Units.inchesToMeters(3));
    encoderBR.setPositionConversionFactor(2 * Math.PI * Units.inchesToMeters(3));
    
    simEncoderL.setDistancePerPulse(2 * Math.PI * (Settings.Drivetrain.kWheelDiameter / 2) / 42);
    simEncoderR.setDistancePerPulse(2 * Math.PI * (Settings.Drivetrain.kWheelDiameter / 2) / 42);

    SimDrivetrain = new DifferentialDrivetrainSim(
        DCMotor.getNEO(Settings.Drivetrain.kGearboxMotorCount),
        Settings.Drivetrain.kGearRatio,
        Settings.Drivetrain.kCenterMomentInertia,
        Settings.Drivetrain.kDriveBaseWeight,
        Units.inchesToMeters(Settings.Drivetrain.kWheelDiameter / 2),
        Units.inchesToMeters(Settings.Drivetrain.kWheelTrackWidth),
        VecBuilder.fill(0.001, 0.001, 0.001, 0.05, 0.05, 0.005, 0.005)
    );

    // if (RobotBase.isSimulation()){
    //   for (Arena.Trajectories.Routine0 paths : Arena.Trajectories.Routine0.values()){
    //     field.getObject(paths.name()).setTrajectory(paths.trajectory);
    //   }
    // }

    SmartDashboard.putData("Field", field);
  }

  /**
   * Drive the robot! Uses traditional two Y-axes to move each side.
   * @param leftSpeed - The speed at which the left side motors should be.
   * @param rightSpeed - The speed at which the right side motors should be.
   */
  public void tankDrive(double leftSpeed, double rightSpeed) {
    drivetrain.tankDrive(leftSpeed, rightSpeed);
  }
  
  /**
   * Drive the robot! Uses more modern X/Y-axes to move and turn seperately.
   * @param xSpeed
   * @param rotation
   */
  public void arcadeDrive(double xSpeed, double rotation) {  
    drivetrain.arcadeDrive(xSpeed, rotation);
  }

  /**
   * Drive the robot! Uses fed voltage inputs to move each side.
   * @param leftVoltage - The voltage at which the left side motors should be.
   * @param rightVoltage - The voltage at which the right side motors should be.
   */
  public void voltageDrive(double leftVoltage, double rightVoltage) {
    MathUtil.clamp(leftVoltage, -RoboRIO.getBatteryVoltage(), RoboRIO.getBatteryVoltage());
    MathUtil.clamp(rightVoltage, -RoboRIO.getBatteryVoltage(), RoboRIO.getBatteryVoltage());

    leftMotors.setVoltage(-leftVoltage);
    rightMotors.setVoltage(-rightVoltage);

    drivetrain.feed();
  }

  /**
   * Forces the motors to stop moving, stopping the robot.
   */
  public void driveStop() {
    drivetrain.stopMotor();
  }

  /**
   * Draw a trajectory in robot simulation.
   * @param trajectory - the trajectory to be drawn
   */
  public void displayPath(Trajectory trajectory) {
    field.getObject("new trajectory").setTrajectory(trajectory);
  }

  // POSITION FUNCTIONS

  public double getLeftPosition() {
    if (RobotBase.isSimulation()) {
      return simEncoderL.getDistance();
    }
    //real encoders might need to be inverted for PID control
    return Settings.Drivetrain.kWheelCircumference * (-encoderFL.getPosition() + -encoderBL.getPosition() / 2.0);
  }
  public double getRightPosition() {
    if (RobotBase.isSimulation()) {
      return simEncoderR.getDistance();
    }
    return Settings.Drivetrain.kWheelCircumference * (encoderFR.getPosition() + encoderBR.getPosition() / 2.0);
  }

  // VELOCITY FUNCTIONS

  public double getLeftVelocity() {
    if (RobotBase.isSimulation()) {
      return simEncoderL.getRate();
    }
    return -Settings.Drivetrain.kWheelCircumference * (encoderFL.getVelocity() + encoderBL.getVelocity() / 120.0);
  }
  public double getRightVelocity() {
    if (RobotBase.isSimulation()) {
      return simEncoderR.getRate();
    }
    return Settings.Drivetrain.kWheelCircumference * (encoderFR.getVelocity() + encoderBR.getVelocity() / 120.0);
  }
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    // double AvgLeftVel = 0.254 * (getLeftVelocity() * 2 * Math.PI * (Settings.Drivetrain.kWheelDiameter / 2)) / 60;
    double AvgLeftVel = getLeftVelocity();
    double AvgRightVel = getRightVelocity();

    return new DifferentialDriveWheelSpeeds(AvgLeftVel, AvgRightVel); // in m/s
  }

  // VOLTAGE FUNCTIONS

  public double getLeftVoltage() {
    return (leftMotors.get() * RoboRIO.getBatteryVoltage());
  }
  public double getRightVoltage() {
    return (rightMotors.get() * RoboRIO.getBatteryVoltage());
  }
  public void setMaxOutput(double maxOutput) {
    drivetrain.setMaxOutput(maxOutput);
  }

  // RESET ENCODERS
  
  public void resetEncoders() {
    encoderFL.setPosition(0);
    encoderBL.setPosition(0);
    encoderFR.setPosition(0);
    encoderBR.setPosition(0);
  }

  public void setCoastMode() {
    motorBL.setIdleMode(IdleMode.kCoast);
    motorBR.setIdleMode(IdleMode.kCoast);
    motorFL.setIdleMode(IdleMode.kCoast);
    motorFR.setIdleMode(IdleMode.kCoast);
  }
  public void setBrakeMode() {
    motorBL.setIdleMode(IdleMode.kBrake);
    motorBR.setIdleMode(IdleMode.kBrake);
    motorFL.setIdleMode(IdleMode.kBrake);
    motorFR.setIdleMode(IdleMode.kBrake);
  }

  // IMU FUNCTIONS + TRAJECTORY 

  public Rotation2d getHeading() {
    return new Rotation2d(-imu.getAngle());
  }
  public double getTurnRate() {
    return -imu.getRate();
  }
  public Pose2d getPose() {
    return odometry.getEstimatedPosition();
  }
  public Pose2d getInvertedPose2d() {
    return new Pose2d(
      odometry.getEstimatedPosition().getTranslation(),
      odometry.getEstimatedPosition().getRotation().rotateBy(Rotation2d.fromDegrees(180))
    );
  }
  public double getPitch() {
    //Pitching down is positive, so now it is negated
    imu.setYawAxis(ADIS16470_IMU.IMUAxis.kX);
    return -imu.getAngle();
  }

  public void zeroIMU() {
    imu.reset();
    imu.calibrate();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Drivetrain/Left Motors Voltage (V)", getLeftVoltage());
    SmartDashboard.putNumber("Drivetrain/Right Motors Voltage (V)", getRightVoltage());

    SmartDashboard.putNumber("Drivetrain/Left Motors Position (Rotations)", getLeftPosition());
    SmartDashboard.putNumber("Drivetrain/Right Motors Position (Rotations)", getRightPosition());

    SmartDashboard.putNumber("Drivetrain/Left Motors Velocity (RPM)", getLeftVelocity());
    SmartDashboard.putNumber("Drivetrain/Right Motors Velocity (RPM)", getRightVelocity());
    
    SmartDashboard.putNumber("Drivetrain/Left Motors Set Speed [-1,1]:", leftMotors.get());
    SmartDashboard.putNumber("Drivetrain/Right Motors Set Speed [-1,1]", rightMotors.get());

    SmartDashboard.putNumber("IMU/Pitch", getPitch());
    SmartDashboard.putString("IMU/Yaw Axis", imu.getYawAxis().toString());
    SmartDashboard.putNumber("IMU/X Angle", imu.getXFilteredAccelAngle());
    SmartDashboard.putNumber("IMU/Y Angle", imu.getYFilteredAccelAngle());

    // SmartDashboard.putNumberArray("Drivetrain/Pose", poseArray);
    
    odometry.update(getHeading(), simEncoderL.getDistance(), simEncoderR.getDistance());
  }

  @Override
  public void simulationPeriodic() {
    // Converts the [-1, 1] PWM signal to voltage by multiplying it by the robot controller voltage.
    SimDrivetrain.setInputs(leftMotors.get() * RoboRIO.getInputVoltage(), rightMotors.get() * RoboRIO.getInputVoltage()); 

    // Advance the model by 20 ms.
    SimDrivetrain.update(.02);
  
    // Update all of our sensors.
    simEncoderL.setDistance(SimDrivetrain.getLeftPositionMeters());
    simEncoderL.setRate(SimDrivetrain.getLeftVelocityMetersPerSecond());
    simEncoderR.setDistance(SimDrivetrain.getRightPositionMeters());
    simEncoderR.setRate(SimDrivetrain.getRightVelocityMetersPerSecond());
    
    imuSim.setGyroAngleX(MathUtil.inputModulus(SimDrivetrain.getHeading().getDegrees(), -180, 180));
    
    field.setRobotPose(SimDrivetrain.getPose());
  }
}
