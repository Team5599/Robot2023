// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.OperatorConstants;

public class Drivetrain extends SubsystemBase {
  Spark motorBL = new Spark(OperatorConstants.motorBL);
  Spark motorFL = new Spark(OperatorConstants.motorFL);
  Spark motorBR = new Spark(OperatorConstants.motorBR);
  Spark motorFR = new Spark(OperatorConstants.motorFR);
  

  /** Creates a new ExampleSubsystem. */
  public Drivetrain() {

    //invert the one of the sides so that they rotate together
    motorBL.setInverted(true);
    motorFL.setInverted(true);

  }
  public void setSpeed(double leftSpeed, double rightSpeed){
    motorBL.set(leftSpeed);
    motorBR.set(rightSpeed);
    motorFL.set(leftSpeed);
    motorFR.set(rightSpeed);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
