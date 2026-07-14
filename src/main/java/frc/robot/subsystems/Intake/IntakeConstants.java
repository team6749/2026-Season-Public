// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

/** Add your docs here. */
public class IntakeConstants {
    public static final int intakeMotorCanId = 27;
    public static final double intakeMotorGearReduction = 1.0 / 0.6817;
    public static final int intakeMotorCurrentLimit = 40;

    public static final int intakePivotMotorCanId = 28;
    public static final double intakePivotMotorGearReduction = (4 / 1) * (4 / 1) * (32 / 14);
    public static final int intakePivotMotorCurrentLimit = 5;
}
