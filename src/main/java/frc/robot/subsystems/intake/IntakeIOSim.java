// Copyright (c) 2026 FRC Team 1701 Robocubs
// http://github.com/Robocubs
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;
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
        var leaderMotorSimState = leaderMotor.getSimState();
        var followerMotorSimState = followerMotor.getSimState();

        var initialPosition = minimumExtension;
        var extensionMotorSim = new ElevatorSim(
                DCMotor.getMinion(numExtensionMotors),
                extensionReduction,
                mass.in(Kilograms),
                distancePerRotation.in(Meters) / (2 * Math.PI),
                minimumExtension.in(Meters),
                maximumExtension.in(Meters),
                false,
                initialPosition.in(Meters));
        var extensionMotor1SimState = extensionMotor1.getSimState();
        extensionMotor1SimState.setRawRotorPosition(
                Rotations.of(initialPosition.in(Meters) / distancePerRotation.in(Meters) * extensionReduction));
        var extensionMotor2SimState = extensionMotor2.getSimState();
        extensionMotor2SimState.setRawRotorPosition(
                Rotations.of(initialPosition.in(Meters) / distancePerRotation.in(Meters) * extensionReduction));

        var rollerInversionValue = rollerMotorInvertedValue == InvertedValue.CounterClockwise_Positive ? 1 : -1;
        var extensionInversionValue = extension1MotorInvertedValue == InvertedValue.CounterClockwise_Positive ? 1 : -1;

        SimNotifier.register(deltaTime -> {
            rollerMotorSim.setInputVoltage(leaderMotorSimState.getMotorVoltage());
            rollerMotorSim.update(deltaTime.in(Seconds));

            var rollerVelocity = rollerMotorSim.getAngularVelocity();

            leaderMotorSimState.addRotorPosition(
                    rollerVelocity.times(rollerReduction).times(deltaTime));
            leaderMotorSimState.setRotorVelocity(rollerVelocity.times(rollerReduction));
            leaderMotorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

            followerMotorSimState.addRotorPosition(
                    rollerVelocity.times(rollerReduction).times(deltaTime));
            followerMotorSimState.setRotorVelocity(rollerVelocity.times(rollerReduction));
            followerMotorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

            extensionMotorSim.setInputVoltage(extensionMotor1SimState.getMotorVoltage() * extensionInversionValue);
            extensionMotorSim.update(deltaTime.in(Seconds));

            var positionMeters = extensionMotorSim.getPositionMeters() * extensionInversionValue;
            var velocityMetersPerSecond = extensionMotorSim.getVelocityMetersPerSecond() * extensionInversionValue;
            var extensionPosition = Rotations.of(positionMeters / distancePerRotation.in(Meters));
            var extensionVelocity = RotationsPerSecond.of(velocityMetersPerSecond / distancePerRotation.in(Meters));
            var isExtended = positionMeters > maximumExtension.in(Meters) - Constants.epsilon;

            extensionMotor1SimState.setRawRotorPosition(extensionPosition.times(extensionReduction));
            extensionMotor1SimState.setRotorVelocity(extensionVelocity.times(extensionReduction));
            extensionMotor1SimState.setSupplyVoltage(RobotController.getBatteryVoltage());
            extensionMotor1SimState.setForwardLimit(isExtended);

            extensionMotor2SimState.setRawRotorPosition(extensionPosition.times(-extensionReduction));
            extensionMotor2SimState.setRotorVelocity(extensionVelocity.times(-extensionReduction));
            extensionMotor2SimState.setSupplyVoltage(RobotController.getBatteryVoltage());
            extensionMotor2SimState.setForwardLimit(isExtended);

            var outputRollerVelocity = rollerVelocity.times(rollerInversionValue);
            simRobotState.updateIntakeSim(
                    Meters.of(positionMeters * extensionInversionValue).gt(minIntakingExtension)
                            && outputRollerVelocity.gt(minIntakingVelocity));
        });
    }
}
