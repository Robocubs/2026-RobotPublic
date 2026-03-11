// Copyright (c) 2026 FRC Team 1701 Robocubs
// http://github.com/Robocubs
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.simulation;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.FieldConstants;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnField;

import static edu.wpi.first.units.Units.*;

public class MapleSimArena extends Arena2026Rebuilt {
    private static final double fuelDiameterMeters = 0.15;

    public MapleSimArena() {
        super(false);
    }

    // Fixes some of the fuel placement from the original
    // Also makes the efficiency mode fill the middle half of the field instead of every few rows
    @Override
    public void placeGamePiecesOnField() {
        blueOutpost.reset();
        redOutpost.reset();

        boolean isOnBlue = DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Blue;

        var minI = isOnBlue || !isInEfficiencyMode ? 0 : 6;
        var maxI = !isOnBlue || !isInEfficiencyMode ? 12 : 6;
        for (var i = minI; i < maxI; i++) {
            var xOffset = (-5.5 + i) * fuelDiameterMeters;
            for (var j = 0; j < 15; j++) {
                var yOffset = -Units.inchesToMeters(90.95) + (j + 0.5) * fuelDiameterMeters;
                var translation = FieldConstants.center.plus(new Translation2d(xOffset, yOffset));
                addGamePiece(new RebuiltFuelOnField(translation));
            }
            for (var j = 0; j < 15; j++) {
                var yOffset = Units.inchesToMeters(90.95) - (j + 0.5) * fuelDiameterMeters;
                var translation = FieldConstants.center.plus(new Translation2d(xOffset, yOffset));
                addGamePiece(new RebuiltFuelOnField(translation));
            }
        }

        if (isOnBlue || !isInEfficiencyMode) {
            for (int x = 0; x < 4; x++) {
                for (int y = 0; y < 6; y++) {
                    addGamePiece(new RebuiltFuelOnField(redDepotBottomRightCorner.plus( // Red and blue are backward
                            new Translation2d(Inches.of(5.991 * x), Inches.of(5.95 * (y + 0.5))))));
                }
            }
        }

        if (!isOnBlue || !isInEfficiencyMode) {
            for (int x = 0; x < 4; x++) {
                for (int y = 0; y < 6; y++) {
                    addGamePiece(new RebuiltFuelOnField(blueDepotBottomRightCorner.plus( // Red and blue are backward
                            new Translation2d(Inches.of(5.991 * x), Inches.of(5.95 * (y + 0.5))))));
                }
            }
        }

        setupValueForMatchBreakdown("CurrentFuelInOutpost");
        setupValueForMatchBreakdown("TotalFuelInOutpost");
        setupValueForMatchBreakdown("TotalFuelInHub");
        setupValueForMatchBreakdown("WastedFuel");
    }
}
