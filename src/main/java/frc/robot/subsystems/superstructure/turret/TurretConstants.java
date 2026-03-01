// Copyright (c) 2026 FRC Team 1701 Robocubs
// http://github.com/Robocubs
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.superstructure.turret;

import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MomentOfInertia;
import frc.robot.util.tuning.LoggedTunableMeasure;

import static edu.wpi.first.units.Units.*;
import static frc.robot.util.UnitsUtil.PoundSquareInches;

public final class TurretConstants {
    public static final int motorId = 27;
    public static final int cancoder1Id = 26;
    public static final int cancoder2Id = 27;

    public static final double reduction = 48.0 / 12.0 * 100.0 / 10.0;
    public static final InvertedValue motorInvertedValue = InvertedValue.CounterClockwise_Positive;
    public static final MomentOfInertia moi = PoundSquareInches.of(10.0 * 4.5 * 4.5);

    public static final double cancoder1Reduction = 100.0 / 13.0;
    public static final double cancoder2Reduction = 100.0 / 17.0;
    public static final Angle cancoder1Offset = Radians.zero();
    public static final Angle cancoder2Offset = Radians.zero();
    public static final Angle crtMatchTolerance = Degrees.of(2.0);
    public static final boolean encoder1Inverted = false;
    public static final boolean encoder2Inverted = false;

    public static final Angle minAngle = Degrees.of(-200);
    public static final Angle maxAngle = Degrees.of(200);
    public static final Angle midAngle = maxAngle.minus(minAngle).div(2);
    public static final boolean maxRangeLessThanOneRotation = maxAngle.in(Rotations) - minAngle.in(Rotations) <= 1.0;

    public static final Transform3d robotToTurret =
            new Transform3d(new Translation3d(Inches.of(10), Inches.of(5), Inches.of(19.625)), Rotation3d.kZero);
    public static final Transform2d robotToTurret2d = new Transform2d(
            robotToTurret.getTranslation().toTranslation2d(),
            robotToTurret.getRotation().toRotation2d());

    public static final LoggedTunableMeasure<AngleUnit, Angle> angleTolerance =
            new LoggedTunableMeasure<>("Turret/AngleTolerance", Degrees.of(2.0));
}
