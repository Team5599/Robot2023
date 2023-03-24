/***************************************************************
                The Sentinels - FRC Team 5599
        Benjamin N. Cardozo High School Robotics Team

    This work is licensed under the terms of the MIT license.
    Copyright (c) 2023 The Sentinels. All rights reserved.
***************************************************************/

package com.sentinels.robot.subsystems.arm;

import com.sentinels.robot.constants.Ports;
import com.sentinels.robot.constants.Settings;
import com.sentinels.robot.util.RoboRIO;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Code to allow the arm to move.
 * 
 * <p>Arm contains:
 * <p>- 1x Falcon 500 Motor on the LEFT side pulley
 * <p>- 1x Falcon 500 Motor on the RIGHT side pulley
 * <p>- 1x NEO Motor on the arm for cascade (extend, retract)
 * 
 * @author Ahmed Osman, Karamat Hasan
 */
public class Arm extends SubsystemBase {

  private final WPI_TalonFX armPivotL = new WPI_TalonFX(Ports.Arm.ARMLEFTPULLEY);
  private final WPI_TalonFX armPivotR = new WPI_TalonFX(Ports.Arm.ARMRIGHTPULLEY);

  private final CANSparkMax armCascade = new CANSparkMax(Ports.Arm.ARMCASCADE, MotorType.kBrushless);
  private final RelativeEncoder cascadeEncoder = armCascade.getEncoder();

  private final MotorControllerGroup armPivotMotors = new MotorControllerGroup(armPivotL, armPivotR);

  public Arm() {
    armPivotL.setNeutralMode(NeutralMode.Brake);
    armPivotR.setNeutralMode(NeutralMode.Brake);
    armPivotL.setInverted(true);

    resetEncoders();
  }
  
  // motor stall is detected by the output current of a motor
  // since both motors will be rotating together, it is assumed that they will also stall together
  // so they use the same method to check if they are stalling

  //may want to add isStalling as an interrupt to its commands
  /*
  public boolean isStalling() {
    boolean currentL = getMotorCurrents()[0] > 90; // according to the revrobotics neo manual, the stall current is at 105 amps
    boolean currentR = getMotorCurrents()[1] > 90;
    //neo motors are rated to reach a max rpm of 5676 and it has a Kv 473(rpm * Kv = max rpm)
    //we need to find the expected speed with the load of the arm and then check if the motor velocity is much lower
    return currentL && currentR;
  }*/

  public void PivotArm(double speed) {
    armPivotMotors.set(speed);
  }

  public void StopArm() {
    armPivotMotors.stopMotor();
  }

  public void CascadeArm(double speed) {
    armCascade.set(speed);
  }

  public void StopCascade() {
    armCascade.stopMotor();
  }

  public void resetEncoders() {
    armPivotL.configFactoryDefault();
    armPivotR.configFactoryDefault();
    armCascade.restoreFactoryDefaults();

    armPivotL.setSelectedSensorPosition(0);
    armPivotR.setSelectedSensorPosition(0);
    cascadeEncoder.setPosition(0);
  }

  // POSITION METHODS (degrees)

  public double getLeftPosition() {
    return armPivotL.getSelectedSensorPosition();
  }
  public double getRightPosition() {
    return armPivotR.getSelectedSensorPosition();
  }
  /**
   * 
   * @return the cascade motor's position
   */
  public double getCascadeMotorPosition() {
    return cascadeEncoder.getPosition();
  }
  /**
   * 
   * @return the distance the cascade has extended
   */
  public double getCascadeExtensionDist(){
    return 2*Math.PI*getCascadeMotorPosition();
  }

  // VELOCITY METHODS (RPM) (converted deg/sec to rpm)

  public double getLeftVelocity() {
    return (armPivotL.getSelectedSensorVelocity() * 6.0);
  }
  public double getRightVelocity() {
    return (armPivotR.getSelectedSensorVelocity() * 6.0);
  }
  /**
   * 
   * @return the angle of the arm using the ticks of the Falcons, assuming there are 1024 ticks per rotation
   * divided by 27 because of the gear ratio
   */
  public double getArmPivotAngle(){
    // return (((armPivotL.getSelectedSensorPosition() % 4096)/4096)/Settings.Arm.kArmPivotGearRatio)*360;
    double encoderPosition = armPivotL.getSelectedSensorPosition();
    double angle = ((double)encoderPosition / (4096  * Settings.Arm.kArmPivotGearRatio)) * 360;
    return angle % 360;
  }
  public double getCascadeVelocity() {
    return cascadeEncoder.getVelocity();
  }

  public double getArmEncoderValue(){
    return armPivotL.getSelectedSensorPosition();
  }

  // VOLTAGE METHODS (V)

  public double getLeftVoltage() {
    return (armPivotL.get() * RoboRIO.getBatteryVoltage());
  }
  public double getRightVoltage() {
    return (armPivotR.get() * RoboRIO.getBatteryVoltage());
  }
  public double getCascadeVoltage() {
    return (armCascade.get() * RoboRIO.getBatteryVoltage());
  }

  // TEMPERATURE METHODS (C)

  public double getLeftTemp() {
    return armPivotL.getTemperature();
  }
  public double getRightTemp() {
    return armPivotR.getTemperature();
  }
  public double getCascadeTemp() {
    return armCascade.getMotorTemperature();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Arm/Left Motor Voltage (V)", getLeftVoltage());
    SmartDashboard.putNumber("Arm/Right Motor Voltage (V)", getRightVoltage());
    SmartDashboard.putNumber("Arm/Cascade Motor Voltage (V)", getCascadeVoltage());

    SmartDashboard.putNumber("Arm/Left Motor Position (Degrees)", getLeftPosition());
    SmartDashboard.putNumber("Arm/Right Motor Position (Degrees)", getRightPosition());
    SmartDashboard.putNumber("Arm/Cascade Motor Position (Degrees)", getCascadeMotorPosition());
    SmartDashboard.putNumber("Arm/Cascade Extension Length", getCascadeExtensionDist());

    SmartDashboard.putNumber("Arm/Left Motor Velocity (RPM)", getLeftVelocity());
    SmartDashboard.putNumber("Arm/Right Motor Velocity (RPM)", getRightVelocity());
    SmartDashboard.putNumber("Arm/Cascade Motor Velocity (RPM)", getCascadeVelocity());

    SmartDashboard.putNumber("Arm/Debug/Left Motor Temperature (C)", getLeftTemp());
    SmartDashboard.putNumber("Arm/Debug/Right Motor Temperature (C)", getRightTemp());
    SmartDashboard.putNumber("Arm/Debug/Cascade Motor Temperature (C)", getCascadeTemp());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
