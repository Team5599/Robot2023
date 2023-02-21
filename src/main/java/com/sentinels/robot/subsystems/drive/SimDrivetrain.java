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
        DCMotor.getNEO(Settings.Drivetrain.GEARBOX_NUM_MOTORS),
        Settings.Drivetrain.GEAR_RATIO,
        Settings.Drivetrain.CENTER_MOMENT_INERTIA,
        Settings.Drivetrain.DRIVEBASE_KG,
        Units.inchesToMeters(Settings.Drivetrain.WHEEL_DIAMETER / 2),
        Units.inchesToMeters(Settings.Drivetrain.WHEEL_TRACK_WIDTH),
        VecBuilder.fill(0.001, 0.001, 0.001, 0.05, 0.05, 0.005, 0.005)
    );
}