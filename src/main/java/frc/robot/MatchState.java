// Copyright (c) 2026 FRC Team 1701 Robocubs
// http://github.com/Robocubs
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import java.util.Optional;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import static edu.wpi.first.units.Units.*;

public class MatchState {
    public enum Phase {
        DISABLED,
        AUTO,
        TRANSITION,
        SHIFT1,
        SHIFT2,
        SHIFT3,
        SHIFT4,
        ENDGAME
    }

    private static final double transitionEnd = 130.0;
    private static final double shift1End = 105.0;
    private static final double shift2End = 80.0;
    private static final double shift3End = 55.0;
    private static final double shift4End = 30.0;

    private final LoggedDashboardChooser<Optional<Alliance>> autoWinnerOverride =
            new LoggedDashboardChooser<>("AutoWinnerOverride");

    private Optional<Alliance> winner = Optional.empty();
    private boolean wonAuto = false;
    private @AutoLogOutput @Getter Phase phase = Phase.DISABLED;
    private @AutoLogOutput @Getter boolean hubActive = false;
    private @AutoLogOutput @Getter Time timeToHubActive = Seconds.zero();
    private @AutoLogOutput @Getter Time timeToHubInactive = Seconds.zero();

    MatchState() {
        autoWinnerOverride.addDefaultOption("Auto", Optional.empty());
        autoWinnerOverride.addOption("Red", Optional.of(Alliance.Red));
        autoWinnerOverride.addOption("Blue", Optional.of(Alliance.Blue));

        // Explicitly reset to Auto on every launch so no persisted NT value carries over.
        NetworkTableInstance.getDefault()
                .getTable("SmartDashboard")
                .getSubTable("AutoWinnerOverride")
                .getEntry("selected")
                .setString("Auto");
    }

    void update() {
        winner = computeAutoWinner();
        wonAuto = winner.isPresent()
                && DriverStation.getAlliance().isPresent()
                && winner.get() == DriverStation.getAlliance().get();
        phase = computePhase();
        hubActive = computeHubActive();
        timeToHubActive = computeTimeToHubActive();
        timeToHubInactive = computeTimeToHubInactive();

        SmartDashboard.putString("AutoWinner", winner.map(Enum::name).orElse("Unknown"));
    }

    private Phase computePhase() {
        if (DriverStation.isDisabled()) {
            return Phase.DISABLED;
        }
        if (DriverStation.isAutonomous()) {
            return Phase.AUTO;
        }
        var matchTime = DriverStation.getMatchTime();
        if (matchTime > transitionEnd) {
            return Phase.TRANSITION;
        }
        if (matchTime > shift1End) {
            return Phase.SHIFT1;
        }
        if (matchTime > shift2End) {
            return Phase.SHIFT2;
        }
        if (matchTime > shift3End) {
            return Phase.SHIFT3;
        }
        if (matchTime > shift4End) {
            return Phase.SHIFT4;
        }
        return Phase.ENDGAME;
    }

    private Optional<Alliance> computeAutoWinner() {
        var override = autoWinnerOverride.get();
        if (override != null && override.isPresent()) {
            return override;
        }

        var gameData = DriverStation.getGameSpecificMessage();
        if (gameData == null || gameData.isEmpty()) {
            return Optional.empty();
        }

        return switch (gameData.charAt(0)) {
            case 'R' -> Optional.of(Alliance.Red);
            case 'B' -> Optional.of(Alliance.Blue);
            default -> Optional.empty();
        };
    }

    private boolean computeHubActive() {
        if (winner.isEmpty()) {
            return true;
        }

        return switch (phase) {
            case DISABLED, AUTO, TRANSITION, ENDGAME -> true;
            case SHIFT1, SHIFT3 -> !wonAuto;
            case SHIFT2, SHIFT4 -> wonAuto;
        };
    }

    private Time computeTimeToHubActive() {
        if (hubActive) {
            return Seconds.zero();
        }

        if (!winner.isPresent()) {
            return Seconds.zero();
        }

        var matchTime = DriverStation.getMatchTime();
        if (wonAuto) {
            // Inactive during Shift 1 or Shift 3
            if (matchTime > shift1End) {
                return Seconds.of(matchTime - shift1End);
            }
            if (matchTime > shift3End) {
                return Seconds.of(matchTime - shift3End);
            }
        } else {
            // Inactive during Shift 2 or Shift 4
            if (matchTime > shift2End) {
                return Seconds.of(matchTime - shift2End);
            }
            if (matchTime > shift4End) {
                return Seconds.of(matchTime - shift4End);
            }
        }

        return Seconds.zero();
    }

    private Time computeTimeToHubInactive() {
        if (!hubActive) {
            return Seconds.zero();
        }

        var matchTime = DriverStation.getMatchTime();

        if (matchTime < 0) {
            return Seconds.zero();
        }

        if (!winner.isPresent()) {
            return Seconds.of(matchTime);
        }

        if (wonAuto) {
            // Active during Transition, Shift 2, Shift 4, and Endgame
            if (matchTime > transitionEnd) {
                return Seconds.of(matchTime - transitionEnd);
            }
            if (matchTime > shift2End) {
                return Seconds.of(matchTime - shift2End);
            }
        } else {
            // Active during Transition, Shift 1, Shift 3, and Endgame
            if (matchTime > shift1End) {
                return Seconds.of(matchTime - shift1End);
            }
            if (matchTime > shift3End) {
                return Seconds.of(matchTime - shift3End);
            }
        }

        return Seconds.of(matchTime);
    }
}
