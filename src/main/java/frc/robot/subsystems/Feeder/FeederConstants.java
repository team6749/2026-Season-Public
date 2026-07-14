package frc.robot.subsystems.Feeder;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Distance;

public class FeederConstants {
    static double gearReduction = 2d / 1d;
    static Distance magicWheelRatio = Inches.of(Math.PI * 2.5 / 6);
}