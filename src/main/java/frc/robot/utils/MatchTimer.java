package frc.robot.utils;

import static edu.wpi.first.units.Units.Seconds;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class MatchTimer {

    boolean isAuto = false;
    boolean isTeleop = false;

    enum TimingSource {
        FMS,
        UseEnabledState
    }

    final Timer timer = new Timer();

    SendableChooser<TimingSource> timerSource = new SendableChooser<TimingSource>();

    public MatchTimer() {
        timerSource.setDefaultOption("USE Enabled State", TimingSource.UseEnabledState);
        timerSource.addOption("USE FMS", TimingSource.FMS);
        SmartDashboard.putData("Timer", timerSource);
    }

    @AutoLogOutput(key = "Timer")
    public Time getMatchTime() {
        switch (timerSource.getSelected()) {
            case FMS:
                return Seconds.of(DriverStation.getMatchTime());
            case UseEnabledState:
                if (isAuto) {
                    return Seconds.of(20 - timer.get());
                }
                if (isTeleop) {
                    return Seconds.of(140 - timer.get());
                }
                return Seconds.of(0);
            default:
                return Seconds.of(0);
        }
    }

    public void periodic() {

        if (DriverStation.isAutonomousEnabled() && isAuto == false) {
            // We are in auto
            timer.reset();
            timer.start();
            isTeleop = false;
            isAuto = true;
        }
        if (DriverStation.isTeleopEnabled() && isTeleop == false) {
            timer.reset();
            timer.start();
            isAuto = false;
            isTeleop = true;
        }
        if (DriverStation.isDisabled()) {
            timer.stop();
            isAuto = false;
            isTeleop = false;
        }
    }

}
