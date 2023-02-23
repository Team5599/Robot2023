package com.sentinels.robot.subsystems.drive;

import com.sentinels.robot.util.RoboRIO;
import com.sentinels.robot.constants.Settings;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SimDrivetrain extends SubsystemBase {

    static final DifferentialDrivetrainSim simDrivetrain = new DifferentialDrivetrainSim(
        DCMotor.getNEO(Settings.Drivetrain.kGearboxMotorCount),
        Settings.Drivetrain.kGearRatio,
        Settings.Drivetrain.kCenterMomentInertia,
        Settings.Drivetrain.kDriveBaseWeight,
        Units.inchesToMeters(Settings.Drivetrain.kWheelDiameter / 2),
        Units.inchesToMeters(Settings.Drivetrain.kWheelTrackWidth),
        VecBuilder.fill(0.001, 0.001, 0.001, 0.05, 0.05, 0.005, 0.005)
    );
}