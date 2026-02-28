// Copyright (c) 2026 FRC Team 1701 Robocubs
// http://github.com/Robocubs
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import java.util.Optional;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import lombok.AccessLevel;
import lombok.Getter;
import lombok.NoArgsConstructor;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import static edu.wpi.first.units.Units.*;

@NoArgsConstructor(access = AccessLevel.PACKAGE)
public class MatchState {
    private static final double transitionEnd = 130.0;
    private static final double shift1End = 105.0;
    private static final double shift2End = 80.0;
    private static final double shift3End = 55.0;
    private static final double shift4End = 30.0;

    private final LoggedNetworkBoolean overrideRedWonAuto =
            new LoggedNetworkBoolean("OverrideRedWonAuto", Constants.currentMode == Constants.Mode.SIM);
    private final LoggedNetworkBoolean overrideBlueWonAuto = new LoggedNetworkBoolean("OverrideBlueWonAuto", false);

    private Optional<Alliance> winner = Optional.empty();
    private boolean wonAuto = false;
    private @AutoLogOutput @Getter boolean hubActive = false;
    private @AutoLogOutput @Getter Time timeToHubActive = Seconds.zero();
    private @AutoLogOutput @Getter Time timeToHubInactive = Seconds.zero();

    void update() {
        winner = computeAutoWinner();
        wonAuto = winner.isPresent()
                && DriverStation.getAlliance().isPresent()
                && winner.get() == DriverStation.getAlliance().get();
        hubActive = computeHubActive(winner);
        timeToHubActive = computeTimeToHubActive();
        timeToHubInactive = computeTimeToHubInactive();

        SmartDashboard.putString("AutoWinner", winner.map(Enum::name).orElse("Unknown"));
        SmartDashboard.putNumber("TimeToActive", timeToHubActive.in(Seconds));
        SmartDashboard.putNumber("TimeToInactive", timeToHubInactive.in(Seconds));
    }

    private Optional<Alliance> computeAutoWinner() {
        if (overrideRedWonAuto.get()) {
            return Optional.of(Alliance.Red);
        }

        if (overrideBlueWonAuto.get()) {
            return Optional.of(Alliance.Blue);
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

    private boolean computeHubActive(Optional<Alliance> winner) {
        if (winner.isEmpty()) {
            return true;
        }

        var matchTime = DriverStation.getMatchTime();

        if (matchTime > transitionEnd) {
            return true;
        }

        if (matchTime > shift1End) {
            return !wonAuto;
        }

        if (matchTime > shift2End) {
            return wonAuto;
        }

        if (matchTime > shift3End) {
            return !wonAuto;
        }

        if (matchTime > shift4End) {
            return wonAuto;
        }

        return true;
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

        if (!winner.isPresent()) {
            return Seconds.zero();
        }

        var matchTime = DriverStation.getMatchTime();

        if (matchTime < 0) {
            return Seconds.zero();
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
