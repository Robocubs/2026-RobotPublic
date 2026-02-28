// Copyright (c) 2026 FRC Team 1701 Robocubs
// http://github.com/Robocubs
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.superstructure.hood;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.StatusSignalCollection;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Servo;
import frc.robot.Constants;
import frc.robot.PhoenixCANBus;
import frc.robot.util.PhoenixUtil;
import frc.robot.util.tuning.LoggedTunableValue;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.superstructure.hood.HoodConstants.*;

public class HoodIOHardware implements HoodIO {
    protected final Servo servo;
    protected final CANcoder cancoder;

    private final CANcoderConfiguration cancoderConfig;

    private final StatusSignalCollection signals = new StatusSignalCollection();
    private final StatusSignal<Angle> positionSignal;
    private final StatusSignal<AngularVelocity> velocitySignal;

    public HoodIOHardware() {
        this.servo = new Servo(servoId);
        this.cancoder = new CANcoder(cancoderId, PhoenixCANBus.canivore());

        cancoderConfig = new CANcoderConfiguration()
                .withMagnetSensor(new MagnetSensorConfigs()
                        .withMagnetOffset(encoderOffset.get())
                        .withSensorDirection(SensorDirectionValue.Clockwise_Positive)
                        .withAbsoluteSensorDiscontinuityPoint(Rotations.of(0.95)));
        PhoenixUtil.tryUntilOk(() -> cancoder.getConfigurator().apply(cancoderConfig));

        positionSignal = cancoder.getAbsolutePosition();
        velocitySignal = cancoder.getVelocity();

        signals.addSignals(positionSignal, velocitySignal);
        signals.setUpdateFrequencyForAll(Constants.mainLoopFrequency);

        cancoder.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(HoodInputs inputs) {
        signals.refreshAll();

        inputs.position = positionSignal.getValue();
        inputs.velocity = velocitySignal.getValue();

        LoggedTunableValue.ifChanged(
                0,
                () -> {
                    cancoderConfig.MagnetSensor.withMagnetOffset(encoderOffset.get());
                    PhoenixUtil.tryUntilOk(() -> cancoder.getConfigurator().apply(cancoderConfig));
                },
                encoderOffset);
    }

    @Override
    public void setServoOutput(double output) {
        // Input is in the range [-1, 1]
        // Servo.set() expects [0, 1] where 0 = full reverse, 0.5 = stopped, 1 = full forward
        // Map [-1, 1] -> [0, 1]
        var servoValue = (output * servoSign + 1.0) / 2.0;
        servoValue = MathUtil.clamp(servoValue, 0.0, 1.0);
        servo.set(servoValue);
    }

    @Override
    public void stop() {
        // 0.5 = stopped for the servo
        servo.set(0.5);
    }
}
