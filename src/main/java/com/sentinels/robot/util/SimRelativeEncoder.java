// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.sentinels.robot.util;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SimRelativeEncoder extends SparkMaxRelativeEncoder {
  final CANSparkMax sparkMax;
  final Type type;
  final int countsPerRev;

  SimRelativeEncoder(CANSparkMax sparkMax, Type type, int countsPerRev) {
    super(sparkMax, type, countsPerRev)
  }
  /** Creates a new SimRelativeEncoder. */

}