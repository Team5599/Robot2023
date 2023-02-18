package com.sentinels.robot.subsystems.vision;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight extends SubsystemBase {

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");

    public Limelight() {
        NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");
    }

    @Override
    public void periodic() {
        //read values periodically
        double x = tx.getDouble(0.0);
        double y = ty.getDouble(0.0);
        double area = ta.getDouble(0.0);

        //post to smart dashboard periodically
        SmartDashboard.putNumber("LimelightX", x);
        SmartDashboard.putNumber("LimelightY", y);
        SmartDashboard.putNumber("LimelightArea", area);
    }
    
    // these will return -191 if there is no game piece present
    public double getTx(){//gets xoffset angle
        return table.getEntry("tx").getDouble(200);// 200 since it is not an angle that can be returned
    }

    public double getTa(){
        return table.getEntry("ta").getDouble(-200);
    }

    // public double getDistance(){       
    //     return 0.0;
    // }
    public double ParllaxDistance(double angle1, double angle2){
        double distance = Math.sin(angle1)/Math.sin(angle2 - angle1);
        return distance;
    }
}