// Copyright (c) 2026 FRC Team 1701 Robocubs
// http://github.com/Robocubs
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.simulation.SimNotifier;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.intake.IntakeConstants.*;

public class IntakeIOSim extends IntakeIOHardware {
    public IntakeIOSim() {
        var initialAngle = Degrees.of(90);
        var gearbox = DCMotor.getKrakenX44Foc(numPivotMotors);
        var motorSim = new SingleJointedArmSim(
                gearbox,
                pivotReduction,
                moi.in(KilogramSquareMeters),
                length.in(Meters),
                minimumAngle.in(Radians),
                maximumAngle.in(Radians),
                true,
                initialAngle.in(Radians),
                0.001,
                0.0);

        var motorSign = motorInvertedValue == InvertedValue.CounterClockwise_Positive ? 1 : -1;
        var cancoderSign = cancoderSensorDirection == SensorDirectionValue.CounterClockwise_Positive ? 1 : -1;

        if (cancoder != null) {
            var cancoderSimState = cancoder.getSimState();
            cancoderSimState.setRawPosition(
                    initialAngle.minus(encoderOffset.get()).times(cancoderSign));
        }

        var motorSimState = pivotMotor.getSimState();
        motorSimState.setMotorType(MotorType.KrakenX44);
        motorSimState.setRawRotorPosition(initialAngle);

        SimNotifier.register(deltaTime -> {
            motorSim.setInputVoltage(motorSimState.getMotorVoltage() * motorSign);
            motorSim.update(deltaTime.in(Seconds));

            var angle = Radians.of(motorSim.getAngleRads());
            var velocity = RadiansPerSecond.of(motorSim.getVelocityRadPerSec());

            if (cancoder != null) {
                var cancoderSimState = cancoder.getSimState();
                cancoderSimState.setRawPosition(angle.minus(encoderOffset.get()).times(cancoderSign));
                cancoderSimState.setVelocity(velocity.times(cancoderSign));
                cancoderSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
            }

            motorSimState.setRawRotorPosition(angle.times(pivotReduction));
            motorSimState.setRotorVelocity(velocity.times(pivotReduction));
            motorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
        });
    }
}
