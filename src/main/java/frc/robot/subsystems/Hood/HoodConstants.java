// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Hood;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;

/** Add your docs here. */
public class HoodConstants {
    public static final int motorCanId = 22;
    public static final double gearReduction = (152.0 / 12.0) * (35.0 / 15.0);
    public static final int motorCurrentLimit = 10;

    public static final Angle maxPosition = Degrees.of(41.7);
    public static final Angle minPosition = Degrees.of(20.0);
    public static final Angle homePosition = minPosition;
}
