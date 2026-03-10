// Copyright (c) 2026 FRC Team 1701 Robocubs
// http://github.com/Robocubs
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;

import static edu.wpi.first.units.Units.*;

public class RobotConstants {
    public static final Mass robotMass = Pounds.of(140);

    public static final Distance robotLength = Inches.of(24.0);
    public static final Distance robotWidth = Inches.of(30.5);

    public static final Distance bumperThickness = Inches.of(2.75);
    public static final Distance robotLengthWithBumpers = robotLength.plus(bumperThickness.times(2));
    public static final Distance robotWidthWithBumpers = robotWidth.plus(bumperThickness.times(2));
    public static final Distance robotCenterToFrontBumper = robotLengthWithBumpers.div(2);
    public static final Distance robotCenterToBackBumper = robotLengthWithBumpers.minus(robotCenterToFrontBumper);
    public static final Distance robotCenterToLeftBumper = robotWidthWithBumpers.div(2);
    public static final Distance robotCenterToRightBumper = robotWidthWithBumpers.minus(robotCenterToLeftBumper);
    public static final Distance robotCenterToHopperExtended =
            robotLength.div(2).plus(Inches.of(12));
    public static final Distance robotCenterToIntakeExtended = robotCenterToHopperExtended.minus(Inches.of(0.875));

    public static final int maxCapacity = 70;
    public static final Distance intakeWidth = robotWidth.minus(Inches.of(1));
}
