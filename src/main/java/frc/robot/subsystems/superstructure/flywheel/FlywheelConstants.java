// Copyright (c) 2026 FRC Team 1701 Robocubs
// http://github.com/Robocubs
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.superstructure.flywheel;

import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;
import frc.robot.util.tuning.LoggedTunableMeasure;

import static edu.wpi.first.units.Units.*;
import static frc.robot.util.UnitsUtil.PoundSquareInches;

public final class FlywheelConstants {
    public static final int leaderId = 30;
    public static final int followerId = 31;

    public static final double gearReduction = 1.0;
    public static final Distance flywheelRadius = Inches.of(1.5);
    public static final MomentOfInertia moiKgMetersSquared =
            PoundSquareInches.of(0.5 * 0.5 * flywheelRadius.in(Meters) * flywheelRadius.in(Meters));

    public static final LoggedTunableMeasure<AngularVelocityUnit, AngularVelocity> velocityTolerance =
            new LoggedTunableMeasure<>("Flywheel/VelocityTolerance", RadiansPerSecond.of(600 * 0.05 / gearReduction));
}
