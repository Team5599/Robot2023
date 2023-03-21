package com.sentinels.robot.commands.armmech.arm;

import java.lang.Math;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;


//this file will extend the arm and lift it up to a certain position for picking up and dropping off the gamepiece 
public class ArmToPosition extends CommandBase{
    // Called when the command is initially scheduled.
  private final double armLength = 36.0;
  private final double height1 = 0.0;
  
  
  public ArmToPosition() {

  }

  @Override
  public void initialize() { //motor angle: returning .25 = 90 degrees, find the angle the motor needs to turn to to get to the given height
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ArmPivot pivot = new ArmPivot(null, null);
  }
    
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
