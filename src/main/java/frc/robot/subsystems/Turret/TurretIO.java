package frc.robot.subsystems.Turret;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public interface TurretIO {
  @AutoLog
  public static class TurretIOInputs {
    public boolean turretConnected = false;
    Angle angle = Radians.of(0);
    AngularVelocity angularVelocity = RadiansPerSecond.of(0);
    Voltage voltTarget = Volts.of(0);
    Current amps = Amps.of(0);

  }

  public default void updateInputs(TurretIOInputs inputs) {
  };

  public default void setClosedLoopGoal(State state) {
  };

  public default void stop() {
  };

  public default void setVolts(Voltage volts) {}
}
