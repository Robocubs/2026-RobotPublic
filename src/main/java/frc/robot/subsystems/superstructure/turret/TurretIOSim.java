// Copyright (c) 2026 FRC Team 1701 Robocubs
// http://github.com/Robocubs
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.superstructure.turret;

import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.simulation.SimNotifier;
import frc.robot.simulation.SimRobotState;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.superstructure.turret.TurretConstants.*;

public class TurretIOSim extends TurretIOHardware {
    DCMotorSim motorSim;

    public TurretIOSim(SimRobotState simRobotState) {
        var gearbox = DCMotor.getKrakenX44Foc(1);
        var plant = LinearSystemId.createDCMotorSystem(gearbox, moi.in(KilogramSquareMeters), reduction);
        motorSim = new DCMotorSim(plant, gearbox);

        var motorSimState = motor.getSimState();
        motorSimState.setMotorType(MotorType.KrakenX44);

        var encoder1SimState = cancoder1.getSimState();
        var encoder2SimState = cancoder2.getSimState();

        SimNotifier.register(deltaTime -> {
            motorSim.setInputVoltage(motorSimState.getMotorVoltage());
            motorSim.update(deltaTime.in(Seconds));

            var position = Radians.of(motorSim.getAngularPositionRad());
            var velocity = RadiansPerSecond.of(motorSim.getAngularVelocityRadPerSec());

            motorSimState.setRawRotorPosition(position.times(reduction));
            motorSimState.setRotorVelocity(velocity.times(reduction));
            motorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

            encoder1SimState.setRawPosition(position.times(cancoder1Reduction).minus(cancoder1Offset));
            encoder1SimState.setVelocity(velocity.times(cancoder1Reduction));
            encoder1SimState.setSupplyVoltage(RobotController.getBatteryVoltage());

            encoder2SimState.setRawPosition(position.times(cancoder2Reduction).minus(cancoder2Offset));
            encoder2SimState.setVelocity(velocity.times(cancoder2Reduction));
            encoder2SimState.setSupplyVoltage(RobotController.getBatteryVoltage());

            simRobotState.setTurretAngle(position);
        });
    }

    @Override
    public void resetPosition(Angle angle) {
        motorSim.setAngle(angle.in(Radians));
        super.resetPosition(angle);
    }
}
