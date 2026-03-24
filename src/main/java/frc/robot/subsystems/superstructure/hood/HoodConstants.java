// Copyright (c) 2026 FRC Team 1701 Robocubs
// http://github.com/Robocubs
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.superstructure.hood;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.measure.Angle;
import frc.robot.util.tuning.LoggedTunableMeasure;

import static edu.wpi.first.units.Units.*;

public final class HoodConstants {
    public static final int servoId = 0; // PWM channel
    public static final int cancoderId = 25;

    public static final double servoSign = -1;
    public static final double servoReduction = 300.0 / 36.0 * 48.0 / 18.0;
    public static final double encoderReduction = 300.0 / 36.0;
    public static final Angle angleAtEncoderZero = Degrees.of(10);
    public static final Angle minAngle = Degrees.of(10);
    public static final Angle maxAngle = Degrees.of(50);

    public static final LoggedTunableMeasure<AngleUnit, Angle> encoderOffset =
            new LoggedTunableMeasure<>("Hood/EncoderOffset", Radians.of(0.0));

    public static final LoggedTunableMeasure<AngleUnit, Angle> angleTolerance =
            new LoggedTunableMeasure<>("Hood/AngleTolerance", Radians.of(0.04));
}
