// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Milliseconds;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Ballistics.BallisticSimulator.ProjectileParameters;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static double gravity = 9.81;// grav accel in m/s^2
  public static Distance turretRadius = Centimeters.of(5); // Distance of the turret from the center. RANDOM NUMBER
                                                           // RIGHT NOW

  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;
  public static final Time simulationTimestep = Milliseconds.of(20);

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static SwerveModulePosition[] kinematics;

  // An 0 array of module positions. Used in localization
  public static SwerveModulePosition[] startingPositions = new SwerveModulePosition[] {
      new SwerveModulePosition(Meters.of(0), Rotation2d.fromDegrees(0)),
      new SwerveModulePosition(Meters.of(0), Rotation2d.fromDegrees(0)),
      new SwerveModulePosition(Meters.of(0), Rotation2d.fromDegrees(0)),
      new SwerveModulePosition(Meters.of(0), Rotation2d.fromDegrees(0))
  };

  public static class FieldConstants {
    public static Distance fieldLength = Inches.of(651.22);
    public static Distance fieldWidth = Inches.of(317.69);

    public static ProjectileParameters fuel = new ProjectileParameters(
        // 0.5lbs
        0.2267,
        // Cow shaped
        0.47,
        // ~5.91in diameter
        0.075,
        // Air density
        1.16);
  }

}
