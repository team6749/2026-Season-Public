package frc.robot.subsystems.LocalizationSubsystem;

import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.LinearAcceleration;

public class AccelerationData {
  LinearAcceleration xAcceleration;
  LinearAcceleration yAcceleration;
  LinearAcceleration zAcceleration;
  AngularAcceleration angularAcceleration;

  public AccelerationData(LinearAcceleration x, LinearAcceleration y, LinearAcceleration z,
      AngularAcceleration angleAccel) {
    xAcceleration = x;
    yAcceleration = y;
    zAcceleration = z;
    angularAcceleration = angleAccel;
  }
}