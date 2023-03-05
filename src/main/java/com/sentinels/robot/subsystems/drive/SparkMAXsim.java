package com.sentinels.robot.subsystems.drive;
import java.net.CacheRequest;

import com.revrobotics.CANSparkMax;
import com.sentinels.robot.util.RoboRIO;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;

public class SparkMAXsim extends CANSparkMax{
    /*
     * Wrapper class for SparMAX
     * near identical copy
     * https://github.com/ligerbots/InfiniteRecharge2020/blob/infiniteSimulator/src/main/java/frc/robot/simulation/SparkMaxWrapper.java
     * 
     * 
     * 
     */
    private SimDevice SparkSim;
    private SimDouble SparkSIMspeed;

    public SparkMAXsim(int deviceID, MotorType type){
        super(deviceID,type);

        SparkSim = SimDevice.create("Spark Max", deviceID);
        if ( SparkSim != null){
            SparkSIMspeed = SparkSim.createDouble("speed",SimDevice.Direction.kInput,0.0);
        }
    }
    // 
    @Override
    public void set(double speed){
        if(SparkSim != null){
            SparkSIMspeed.set(speed);
        }
        else super.set(speed);
    }
    @Override
    public double get(){
        if(SparkSim != null){
            return SparkSIMspeed.get();
        }
        else return super.get();
    }
    @Override
    public void setVoltage(double outputVolts){
        if (SparkSim != null){
            set(outputVolts / RoboRIO.getBatteryVoltage());
        } else {
            super.setVoltage(outputVolts);
        }        
    }
}
