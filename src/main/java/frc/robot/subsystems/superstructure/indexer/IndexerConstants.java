// Copyright (c) 2026 FRC Team 1701 Robocubs
// http://github.com/Robocubs
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.superstructure.indexer;

import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import frc.robot.util.tuning.LoggedTunableMeasure;

import static edu.wpi.first.units.Units.*;

public final class IndexerConstants {
    public static final int motorId = 28;

    public static final double gearReduction = 9.0;
    public static final Distance wheelRadius = Inches.of(4.0);

    public static final LoggedTunableMeasure<AngularVelocityUnit, AngularVelocity> velocityTolerance =
            new LoggedTunableMeasure<>("Indexer/VelocityTolerance", RadiansPerSecond.of(600 * 0.05 / gearReduction));
}
