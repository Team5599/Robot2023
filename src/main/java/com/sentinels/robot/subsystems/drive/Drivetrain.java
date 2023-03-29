/***************************************************************
                The Sentinels - FRC Team 5599
        Benjamin N. Cardozo High School Robotics Team

    This work is licensed under the terms of the MIT license.
    Copyright (c) 2023 The Sentinels. All rights reserved.
***************************************************************/

package com.sentinels.robot.subsystems.drive;

import com.sentinels.robot.constants.Ports;
import com.sentinels.robot.constants.Settings;
import com.sentinels.robot.constants.Arena;
import com.sentinels.robot.util.RoboRIO;
import com.sentinels.robot.util.SparkMAXsim;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

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
import edu.wpi.first.wpilibj.ADIS16448_IMU.IMUAxis;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.simulation.ADIS16470_IMUSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
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

  private DifferentialDrivetrainSim TestDrivetrain;
  private final DifferentialDrivePoseEstimator odometry = new DifferentialDrivePoseEstimator(
    Settings.Drivetrain.KINEMATICS, getHeading(), getLeftPosition(), getRightPosition(), new Pose2d()
  );

  public Drivetrain() {
    // Resetting motor settings to default factory settings
    motorFL.restoreFactoryDefaults();
    motorBL.restoreFactoryDefaults();
    motorFR.restoreFactoryDefaults();
    motorBR.restoreFactoryDefaults();

    // Set a max output to avoid damage
    setMaxOutput(0.95);
    
    // Zeroing and calibrating IMU
    zeroIMU();

    // Zeroing encoder positions
    resetEncoders();

    // Set velocity and position factors

    encoderFL.setPositionConversionFactor(2 * Math.PI * Units.inchesToMeters(3));
    encoderBL.setPositionConversionFactor(2 * Math.PI * Units.inchesToMeters(3));
    encoderFR.setPositionConversionFactor(2 * Math.PI * Units.inchesToMeters(3));
    encoderBR.setPositionConversionFactor(2 * Math.PI * Units.inchesToMeters(3));

    // Invert the one of the sides so that they rotate synonymously in one direction
    leftMotors.setInverted(true);
    
    simEncoderL.setDistancePerPulse(2 * Math.PI * (Settings.Drivetrain.kWheelDiameter / 2) / 42);
    simEncoderR.setDistancePerPulse(2 * Math.PI * (Settings.Drivetrain.kWheelDiameter / 2) / 42);

    TestDrivetrain = new DifferentialDrivetrainSim(
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
    MathUtil.clamp(leftVoltage, -RoboRIO.getBatteryVoltage() , RoboRIO.getBatteryVoltage());
    MathUtil.clamp(rightVoltage, -RoboRIO.getBatteryVoltage() , RoboRIO.getBatteryVoltage());
    leftMotors.setVoltage(-leftVoltage);
    rightMotors.setVoltage(-rightVoltage);

    drivetrain.feed();
  }
  public void driveStop() {
    drivetrain.stopMotor();
  }
  /**
   * Draw a trajectory in the simulation 
   * @param trajectory - the trajectory to be drawm
   * 
   */
  public void displayPath(Trajectory trajectory){
    field.getObject("new trajectory").setTrajectory(trajectory);
  }

  // POSITION FUNCTIONS

  public double getLeftPosition() {
    if(RobotBase.isSimulation()){
      return simEncoderL.getDistance();
    }
    //real encoders might need to be inverted for PID control
    return Settings.Drivetrain.kWheelCircumference * (-encoderFL.getPosition() + -encoderBL.getPosition() / 2.0);
  }
  public double getRightPosition() {
    if(RobotBase.isSimulation()){
      return simEncoderR.getDistance();
    }
    return Settings.Drivetrain.kWheelCircumference * (encoderFR.getPosition() + encoderBR.getPosition() / 2.0);
  }

  // VELOCITY FUNCTIONS
  public double getLeftVelocity() {
    if (RobotBase.isSimulation()){
      return simEncoderL.getRate();
    }
    return -Settings.Drivetrain.kWheelCircumference * (encoderFL.getVelocity() + encoderBL.getVelocity() / 120.0);
  }
  public double getRightVelocity() {
    // negate so both velocities are positive
    if (RobotBase.isSimulation()){
      return simEncoderR.getRate();
    }
    return Settings.Drivetrain.kWheelCircumference * (encoderFR.getVelocity() + encoderBR.getVelocity() / 120.0);
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds(){
    // double AvgLeftVel = 0.254 * (getLeftVelocity() * 2 * Math.PI * (Settings.Drivetrain.kWheelDiameter / 2)) / 60;
    double AvgLeftVel = getLeftVelocity();
    double AvgRightVel = getRightVelocity();
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
  public void zeroIMU() {
    imu.reset();
    imu.calibrate();
  }
  public Pose2d getPose() {
    return odometry.getEstimatedPosition();
  }
  public double getPitch(){
    //Pitching down is positive, so now it is negated
    imu.setYawAxis(edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis.kX);
    return -imu.getAngle();
  }

  public Pose2d getInvertedPose2d(){
    return new Pose2d(
      odometry.getEstimatedPosition().getTranslation(),
      odometry.getEstimatedPosition().getRotation().rotateBy(Rotation2d.fromDegrees(180))
    );
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Drivetrain/Left Motors Voltage (V)", getLeftVoltage());
    SmartDashboard.putNumber("Drivetrain/Right Motors Voltage (V)", getRightVoltage());

    SmartDashboard.putNumber("Drivetrain/Left Motors Position (Rotations)", getLeftPosition());
    SmartDashboard.putNumber("Drivetrain/Right Motors Position (Rotations)", getRightPosition());

    SmartDashboard.putNumber("Drivetrain/Left Motors Velocity (RPM)", getLeftVelocity());
    SmartDashboard.putNumber("Drivetrain/Right Motors Velocity (RPM)", getRightVelocity());
    
    SmartDashboard.putNumber("Drivetrain/Left get:", leftMotors.get());
    SmartDashboard.putNumber("Drivetrain/Right get:", rightMotors.get());

    SmartDashboard.putNumber("IMU/angle",getPitch());
    SmartDashboard.putString("IMU/yaw axis",imu.getYawAxis().toString());
    SmartDashboard.putNumber("IMU/x angle",imu.getXFilteredAccelAngle());
    SmartDashboard.putNumber("IMU/y angle",imu.getYFilteredAccelAngle());

    // SmartDashboard.putNumberArray("Drivetrain/Pose", poseArray);
    
    // This will get the simulated sensor readings that we set
    // in the previous article while in simulation, but will use
    // real values on the robot itself.
    odometry.update(getHeading(), simEncoderL.getDistance(), simEncoderR.getDistance());

    // odometry.update(getHeading(), )
  }

  @Override
  public void simulationPeriodic() {
    // Set the inputs to the system. Note that we need to convert
    // the [-1, 1] PWM signal to voltage by multiplying it by the
    // robot controller voltage.
    TestDrivetrain.setInputs(leftMotors.get() * RoboRIO.getInputVoltage(), rightMotors.get() * RoboRIO.getInputVoltage());    
    // Advance the model by 20 ms. Note that if you are running this
    // subsystem in a separate thread or have changed the nominal timestep
    // of TimedRobot, this value needs to match it.
    TestDrivetrain.update(.02);
  
    // Update all of our sensors.
    simEncoderL.setDistance(TestDrivetrain.getLeftPositionMeters());
    simEncoderL.setRate(TestDrivetrain.getLeftVelocityMetersPerSecond());
    simEncoderR.setDistance(TestDrivetrain.getRightPositionMeters());
    simEncoderR.setRate(TestDrivetrain.getRightVelocityMetersPerSecond());
    
    imuSim.setGyroAngleX(MathUtil.inputModulus(TestDrivetrain.getHeading().getDegrees(), -180, 180));
    field.setRobotPose(TestDrivetrain.getPose());
  }
}
