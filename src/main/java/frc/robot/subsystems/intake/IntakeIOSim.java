// Copyright (c) 2026 FRC Team 1701 Robocubs
// http://github.com/Robocubs
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.simulation.SimNotifier;
import frc.robot.simulation.SimRobotState;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.intake.IntakeConstants.*;

public class IntakeIOSim extends IntakeIOHardware {
    private static final AngularVelocity minIntakingVelocity = RotationsPerSecond.of(1);
    private static final Distance minIntakingExtension = maximumExtension.minus(Inches.of(0.5));

    public IntakeIOSim(SimRobotState simRobotState) {
        var rollerGearbox = DCMotor.getKrakenX60(1);
        var rollerPlant = LinearSystemId.createFlywheelSystem(rollerGearbox, 0.0001, rollerReduction);
        var rollerMotorSim = new FlywheelSim(rollerPlant, rollerGearbox);
        var rollerMotorSimState = leaderMotor.getSimState();

        var initialPosition = retractedPosition;
        var extensionMotorSim = new ElevatorSim(
                DCMotor.getMinion(numExtensionMotors),
                extensionReduction,
                mass.in(Kilograms),
                distancePerRotation.in(Meters) / (2 * Math.PI),
                minimumExtension.in(Meters),
                maximumExtension.in(Meters),
                false,
                initialPosition.in(Meters));
        var extensionMotorSimState = extensionMotor.getSimState();
        extensionMotorSimState.setRawRotorPosition(
                Rotations.of(initialPosition.in(Meters) / distancePerRotation.in(Meters) * extensionReduction));

        SimNotifier.register(deltaTime -> {
            rollerMotorSim.setInputVoltage(rollerMotorSimState.getMotorVoltage());
            rollerMotorSim.update(deltaTime.in(Seconds));

            var rollerVelocity = rollerMotorSim.getAngularVelocity().times(rollerReduction);

            rollerMotorSimState.addRotorPosition(rollerVelocity.times(deltaTime));
            rollerMotorSimState.setRotorVelocity(rollerVelocity);
            rollerMotorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

            extensionMotorSim.setInputVoltage(extensionMotorSimState.getMotorVoltage());
            extensionMotorSim.update(deltaTime.in(Seconds));

            var extensionPosition =
                    Rotations.of(extensionMotorSim.getPositionMeters() / distancePerRotation.in(Meters));
            var extensionVelocity = RotationsPerSecond.of(
                    extensionMotorSim.getVelocityMetersPerSecond() / distancePerRotation.in(Meters));

            extensionMotorSimState.setRawRotorPosition(extensionPosition.times(extensionReduction));
            extensionMotorSimState.setRotorVelocity(extensionVelocity.times(extensionReduction));
            extensionMotorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

            simRobotState.updateIntakeSim(
                    Meters.of(extensionMotorSim.getPositionMeters()).gt(minIntakingExtension)
                            && rollerVelocity.gt(minIntakingVelocity));
        });
    }
}
