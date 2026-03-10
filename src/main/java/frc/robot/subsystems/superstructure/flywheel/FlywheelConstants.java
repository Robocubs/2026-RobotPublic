// Copyright (c) 2026 FRC Team 1701 Robocubs
// http://github.com/Robocubs
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.superstructure.flywheel;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
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
    public static final MomentOfInertia moi =
            PoundSquareInches.of(0.5 * 3.0 * flywheelRadius.in(Inches) * flywheelRadius.in(Inches));

    public static final double rollerReduction = 18.0 / 30.0 * 22.0 / 22.0;
    public static final Distance rollerRadius = Inches.of(1.0);

    public static final Transform3d turretToFlywheel =
            new Transform3d(Inches.of(2.5), Inches.zero(), Inches.of(6.035), Rotation3d.kZero);
    public static final Distance openingDistance = Inches.of(4.91);

    public static final LoggedTunableMeasure<AngularVelocityUnit, AngularVelocity> velocityTolerance =
            new LoggedTunableMeasure<>("Flywheel/VelocityTolerance", RadiansPerSecond.of(600 * 0.05 / gearReduction));
}
