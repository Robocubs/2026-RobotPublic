// Copyright (c) 2026 FRC Team 1701 Robocubs
// http://github.com/Robocubs
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Velocity;
import frc.robot.util.tuning.LoggedTunableMeasure;

import static edu.wpi.first.units.Units.*;
import static frc.robot.util.UnitsUtil.PoundSquareInches;

public final class IntakeConstants {
    public static final int rollerMotorId = 20;
    public static final int pivotMotorId = 21;
    public static final int cancoderId = 20;

    public static final int numRollerMotors = 1;
    public static final double rollerReduction = 4.0;
    public static final int numPivotMotors = 1;
    public static final double pivotReduction = 100.0;
    public static final Mass mass = Pounds.of(10);
    public static final Distance length = Inches.of(15);
    public static final MomentOfInertia moi =
            PoundSquareInches.of(0.7 * mass.in(Pounds) * length.in(Inches) * length.in(Inches));

    public static final InvertedValue motorInvertedValue = InvertedValue.CounterClockwise_Positive;
    public static final SensorDirectionValue cancoderSensorDirection = SensorDirectionValue.Clockwise_Positive;
    public static final FeedbackSensorSourceValue feedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    public static final LoggedTunableMeasure<AngleUnit, Angle> encoderOffset =
            new LoggedTunableMeasure<>("Intake/EncoderOffset", Radians.of(0));

    public static final Angle angleTolerance = Degrees.of(5);
    public static final Angle maximumAngle = Degrees.of(95.25);
    public static final Angle minimumAngle = Degrees.of(-68);

    public static final AngularVelocity maximumVelocity = RotationsPerSecond.of(1.0);
    public static final AngularAcceleration maximumAcceleration = maximumVelocity.div(Seconds.of(0.2));
    public static final Velocity<AngularAccelerationUnit> maximumJerk = maximumAcceleration.div(Seconds.of(0.05));
    public static final double reduction = 1;

    public static final Angle angleDeployed = Degrees.of(90);
    public static final Angle angleRetracted = Degrees.of(0);
}
