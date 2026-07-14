package frc.robot.subsystems.Turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;

public class TurretConstants {
    public static double gearReduction = (115d / 36d) * (3d / 1d);
    public static AngularVelocity maxTurretVelo = DegreesPerSecond.of(720);
    public static AngularAcceleration maxTurretAccel = DegreesPerSecondPerSecond.of(1500);
    public static final int motorCurrentLimit = 70;

    public static Angle minAngle = Degrees.of(-115);
    public static Angle maxAngle = Degrees.of(135);

    // The rotation between robot relative and hardware relative fields.
    public static Rotation2d turretAxisOffset = Rotation2d.fromRadians(Math.PI);

}
