// Copyright (c) 2026 FRC Team 1701 Robocubs
// http://github.com/Robocubs
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.superstructure.kicker;

import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.util.tuning.LoggedTunableMeasure;

import static edu.wpi.first.units.Units.RadiansPerSecond;

public final class KickerConstants {
    public static final int motorId = 32;

    public static final double gearReduction = 4.0;

    public static final LoggedTunableMeasure<AngularVelocityUnit, AngularVelocity> velocityTolerance =
            new LoggedTunableMeasure<>("Kicker/VelocityTolerance", RadiansPerSecond.of(600 * 0.05 / gearReduction));
}
