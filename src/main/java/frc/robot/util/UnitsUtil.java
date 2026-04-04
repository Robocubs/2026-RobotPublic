// Copyright (c) 2026 FRC Team 1701 Robocubs
// http://github.com/Robocubs
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.AngularMomentumUnit;
import edu.wpi.first.units.LinearMomentumUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MomentOfInertiaUnit;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

import static edu.wpi.first.units.Units.*;

public class UnitsUtil {
    public static final Angle piRadians = Radians.of(Math.PI);

    public static final LinearMomentumUnit PoundInchesPerSecond = Pounds.mult(InchesPerSecond);
    public static final AngularMomentumUnit PoundInchesSquaredPerSecond = PoundInchesPerSecond.mult(Inches);
    public static final MomentOfInertiaUnit PoundSquareInches = PoundInchesSquaredPerSecond.per(RadiansPerSecond);

    public static AngularVelocity toAngularVelocity(LinearVelocity linearVelocity, Distance radius) {
        return RadiansPerSecond.of(linearVelocity.in(MetersPerSecond) / radius.in(Meters));
    }

    public static <U extends Unit, M extends Measure<U>> M inputModulus(M input, M minimumInput, M maximumInput) {
        var unit = input.unit();
        @SuppressWarnings("unchecked")
        var result = (M) unit.of(MathUtil.inputModulus(input.in(unit), minimumInput.in(unit), maximumInput.in(unit)));
        return result;
    }
}
