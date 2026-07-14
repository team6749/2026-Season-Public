package frc.robot.utils;

import static edu.wpi.first.units.Units.Seconds;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class HubTimer {

    Alert invalidGameDataAlert = new Alert("Game Data is Invalid!", AlertType.kError);

    SendableChooser<Alliance> overrideStartingAllianceChooser = new SendableChooser<Alliance>();

    MatchTimer fieldTimer;

    public HubTimer(MatchTimer matchTimer) {
        this.fieldTimer = matchTimer;

        overrideStartingAllianceChooser.setDefaultOption("Use Game Data", null);
        overrideStartingAllianceChooser.addOption("Red", Alliance.Red);
        overrideStartingAllianceChooser.addOption("Blue", Alliance.Blue);
        SmartDashboard.putData("Auto Alliance Winner Override", overrideStartingAllianceChooser);
    }

    // Positive is before the shift, negative is after the shift starts
    @AutoLogOutput(key = "hubTimer")
    public Time getTimeUntilNextActiveShift() {
        Time matchTime = fieldTimer.getMatchTime();
        // Hub is always enabled in autonomous.
        if (!DriverStation.isTeleopEnabled()) {
            return Seconds.of(0);
        }

        // Hub is always enabled in transition state
        if (matchTime.in(Seconds) > 130) {
            return Seconds.of(0);
        }

        String gameData = DriverStation.getGameSpecificMessage();
        boolean usActiveFirst = false;

        boolean blueInactiveFirst = true;
        if (overrideStartingAllianceChooser.getSelected() != null) {
            blueInactiveFirst = overrideStartingAllianceChooser.getSelected() == Alliance.Blue;
        } else {
            invalidGameDataAlert.set(false);
            // If we have no game data, we cannot compute, assume hub is active, as its
            // likely early in teleop.
            // what the heck FIRST.
            if (gameData.isEmpty()) {
                return Seconds.of(0);
            }
            switch (gameData.charAt(0)) {
                case 'R' -> blueInactiveFirst = false;
                case 'B' -> blueInactiveFirst = true;
                default -> {
                    // If we have invalid game data, assume hub is active.
                    invalidGameDataAlert.set(true);
                    return Seconds.of(0);
                }
            }
        }
        // TODO make a standard DriverStation GetAlliance with an override.
        if (blueInactiveFirst && DriverStation.getAlliance().get() == Alliance.Blue) {
            usActiveFirst = false;
        } else if (!blueInactiveFirst && DriverStation.getAlliance().get() == Alliance.Red) {
            usActiveFirst = false;
        } else {
            usActiveFirst = true;
        }

        if (matchTime.in(Seconds) < 30 && usActiveFirst) {
            // Endgame
            return Seconds.of(0);
        }
        if (matchTime.in(Seconds) < 55 && usActiveFirst) {
            // Lose shift 4
            return Seconds.of(matchTime.in(Seconds) - 30);
        }
        if (matchTime.in(Seconds) < 55 && !usActiveFirst) {
            // Win shift 4
            return Seconds.of(matchTime.in(Seconds) - 55);
        }
        if (matchTime.in(Seconds) < 80 && usActiveFirst) {
            // Lose shift 3
            return Seconds.of(matchTime.in(Seconds) - 80);
        }
        if (matchTime.in(Seconds) < 80 && !usActiveFirst) {
            // Win shift 3
            return Seconds.of(matchTime.in(Seconds) - 55);
        }
        if (matchTime.in(Seconds) < 105 && usActiveFirst) {
            // Lose shift 2
            return Seconds.of(matchTime.in(Seconds) - 80);
        }
        if (matchTime.in(Seconds) < 105 && !usActiveFirst) {
            // Win shift 2
            return Seconds.of(matchTime.in(Seconds) - 105);
        }
        if (matchTime.in(Seconds) < 130 && usActiveFirst) {
            // Lose shift 1
            return Seconds.of(matchTime.in(Seconds) - 130);
        }
        if (matchTime.in(Seconds) < 130 && !usActiveFirst) {
            // Win shift 1
            return Seconds.of(matchTime.in(Seconds) - 105);
        } else {
            return Seconds.of(0);
        }
    }

}
