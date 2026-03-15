// Copyright (c) 2026 FRC Team 1701 Robocubs
// http://github.com/Robocubs
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;

import static edu.wpi.first.units.Units.*;

public final class IntakeConstants {
    public static final int leaderMotorId = 20;
    public static final int followerMotorId = 19;
    public static final int extensionMotorId = 21;

    public static final int numRollerMotors = 1;
    public static final double rollerReduction = 2.0;
    public static final int numExtensionMotors = 1;
    public static final double extensionReduction = 12.0;
    public static final Mass mass = Kilograms.of(5);
    public static final Distance distancePerRotation = Inches.of(Math.PI / 10.0 * 20.0);

    public static final InvertedValue extensionMotorInvertedValue = InvertedValue.CounterClockwise_Positive;
    public static final InvertedValue leaderMotorInvertedValue = InvertedValue.Clockwise_Positive;
    public static final InvertedValue followerMotorInvertedValue =
            leaderMotorInvertedValue == InvertedValue.CounterClockwise_Positive
                    ? InvertedValue.Clockwise_Positive
                    : InvertedValue.CounterClockwise_Positive;

    public static final Distance extensionTolerance = Inches.of(0.5);
    public static final Distance maximumExtension = Inches.of(12.0);
    public static final Distance minimumExtension = Inches.of(0.0);

    public static final LinearVelocity maximumLinearVelocity = InchesPerSecond.of(48.0);
    public static final LinearAcceleration maximumLinearAcceleration = InchesPerSecondPerSecond.of(100.0);

    public static final Distance extendedPosition = Inches.of(12.0);
    public static final Distance partiallyExtendedPosition = Inches.of(4.0);
    public static final Distance retractedPosition = Inches.of(0);
}
