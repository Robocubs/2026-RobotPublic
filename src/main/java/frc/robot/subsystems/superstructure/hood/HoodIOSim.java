// Copyright (c) 2026 FRC Team 1701 Robocubs
// http://github.com/Robocubs
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.superstructure.hood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.simulation.SimNotifier;
import frc.robot.simulation.SimRobotState;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.superstructure.hood.HoodConstants.*;
import static frc.robot.util.UnitsUtil.PoundSquareInches;

public class HoodIOSim implements HoodIO {
    private Voltage appliedVoltage = Volts.zero();
    private Angle currentAngle = minAngle;
    private AngularVelocity currentAngularVelocity = RadiansPerSecond.zero();

    public HoodIOSim(SimRobotState simRobotState) {
        var gearbox = new DCMotor(
                6,
                PoundInches.of(77.90 / 16.0).in(NewtonMeters),
                2.9,
                0.5,
                Degrees.of(60.0).div(Seconds.of(0.043)).in(RadiansPerSecond),
                1);
        var armSim = new SingleJointedArmSim(
                gearbox,
                servoReduction,
                PoundSquareInches.of(1.0 * 7 * 7).in(KilogramSquareMeters),
                7.0,
                minAngle.in(Radians),
                maxAngle.in(Radians),
                true,
                currentAngle.in(Radians));

        SimNotifier.register(deltaTime -> {
            armSim.setInputVoltage(appliedVoltage.in(Volts));
            armSim.update(deltaTime.in(Seconds));

            currentAngle = Radians.of(armSim.getAngleRads());
            currentAngularVelocity = RadiansPerSecond.of(armSim.getVelocityRadPerSec());

            simRobotState.setHoodAngle(currentAngle);
        });
    }

    @Override
    public void updateInputs(HoodInputs inputs) {
        inputs.position = currentAngle.minus(angleAtEncoderZero).times(encoderReduction);
        inputs.velocity = currentAngularVelocity.times(encoderReduction);
    }

    @Override
    public void setServoOutput(double output) {
        appliedVoltage = Volts.of(MathUtil.clamp(output, -1.0, 1.0) * 6.0);
    }

    @Override
    public void stop() {
        appliedVoltage = Volts.zero();
    }
}
