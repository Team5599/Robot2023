package com.sentinels.robot.subsystems.arm;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.sentinels.robot.constants.Ports;

public class Arm extends SubsystemBase {
  Spark ArmL = new Spark(Ports.Arm.ARMLEFT);
  Spark ArmR = new Spark(Ports.Arm.ARMRIGHT);
  Spark ArmPulley = new Spark(Ports.Arm.ARMPULLEY);
  MotorControllerGroup ArmMotors = new MotorControllerGroup(ArmL, ArmR);
    // the left is inverted so that they rotate together. they are kept as group so that they change speeds at the same time
    public Arm() {
      ArmL.setInverted(true);
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
  // these do not detect if the arm is fully extended, fix this asap
  public void ExtendArm(){
    ArmMotors.set(0.5);
  }
  public void RetractArm(){
    ArmMotors.set(-0.5);
  }
  public void StopArm(){
    ArmMotors.set(0);
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
