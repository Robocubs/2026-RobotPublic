// Copyright (c) 2026 FRC Team 1701 Robocubs
// http://github.com/Robocubs
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.simulation.SimNotifier;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.intake.IntakeConstants.*;

public class IntakeIOSim extends IntakeIOHardware {
    public IntakeIOSim() {
        var initialPosition = retractedPosition;
        var motorSim = new ElevatorSim(
                DCMotor.getKrakenX44Foc(numExtensionMotors),
                extensionReduction,
                mass.in(Kilograms),
                distancePerRotation.in(Meters) / (2 * Math.PI),
                minimumExtension.in(Meters),
                maximumExtension.in(Meters),
                false,
                initialPosition.in(Meters));

        var motorSimState = extensionMotor.getSimState();
        motorSimState.setRawRotorPosition(
                Rotations.of(initialPosition.in(Meters) / distancePerRotation.in(Meters) * extensionReduction));

        SimNotifier.register(deltaTime -> {
            motorSim.setInputVoltage(motorSimState.getMotorVoltage());
            motorSim.update(deltaTime.in(Seconds));

            var position = Rotations.of(motorSim.getPositionMeters() / distancePerRotation.in(Meters));
            var velocity =
                    RotationsPerSecond.of(motorSim.getVelocityMetersPerSecond() / distancePerRotation.in(Meters));

            Logger.recordOutput("Intake/SimExtensionPosition", position);
            Logger.recordOutput("Intake/SimExtensionVelocity", velocity);

            motorSimState.setRawRotorPosition(position.times(extensionReduction));
            motorSimState.setRotorVelocity(velocity.times(extensionReduction));
            motorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
        });
    }
}
