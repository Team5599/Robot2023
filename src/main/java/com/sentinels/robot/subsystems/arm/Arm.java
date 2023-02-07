package com.sentinels.robot.subsystems.arm;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.sentinels.robot.constants.Ports;

public class Arm extends SubsystemBase {
  CANSparkMax ArmL = new CANSparkMax(Ports.Arm.ARMLEFT,MotorType.kBrushless);
  CANSparkMax ArmR = new CANSparkMax(Ports.Arm.ARMRIGHT,MotorType.kBrushless);
  CANSparkMax ArmPulley = new CANSparkMax(Ports.Arm.ARMPULLEY,MotorType.kBrushless);
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
  // motor stall is detected by the output current of a motor
  // since both motors will be rotating together, it is assumed that they will also stall together
  // so they use the same method to check if they are stalling
  
  // the left motors currents is the first in the index, the right motor is next in the index
  public double[] getMotorCurrents(){
    double[] motorCurrents = {ArmL.getOutputCurrent(),ArmR.getOutputCurrent()};
    return motorCurrents; 
  }

  //may want to add isStalling as an interrupt to its commands
  public boolean isStalling(){
    boolean currentL = getMotorCurrents()[0] > 105; // according to the revrobotics neo manual, the stall current is at 105 amps
    boolean currentR = getMotorCurrents()[1] > 105;

    //because I cannot find a to get the actual rpm of the motor, i am only checking the current in the motors
    return currentL && currentR;
  }

  public void ExtendArm(){
    if(isStalling()){
      return;
    }
    ArmMotors.set(0.5);
  }
  public void RetractArm(){
    if(isStalling()){
      return;
    }
    ArmMotors.set(-0.5);
  }
  public void StopArm(){
    ArmMotors.set(0);
  }

  // the pulley may also need its own stall checks
  public void ElevatorRaise(){
    ArmPulley.set(0.3);
  }
  public void ElevatorLower(){
    ArmPulley.set(-0.3);
  }
  public void ElevatorStop(){
    ArmPulley.set(0);
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
