// Copyright (c) 2026 FRC Team 1701 Robocubs
// http://github.com/Robocubs
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.superstructure.kicker;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.StatusSignalCollection;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.PhoenixCANBus;
import frc.robot.util.PhoenixUtil;
import frc.robot.util.tuning.LoggedTunableNumber;
import frc.robot.util.tuning.LoggedTunableValue;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.superstructure.kicker.KickerConstants.*;

public class KickerIOHardware implements KickerIO {
    private static final LoggedTunableNumber kS = new LoggedTunableNumber("Kicker/kS", 0.30348);
    private static final LoggedTunableNumber kV =
            new LoggedTunableNumber("Kicker/kV", Units.rotationsToRadians(0.078116));
    private static final LoggedTunableNumber kA =
            new LoggedTunableNumber("Kicker/kA", Units.rotationsToRadians(0.0040192));
    private static final LoggedTunableNumber kP =
            new LoggedTunableNumber("Kicker/kP", Units.rotationsToRadians(0.073004));
    private static final LoggedTunableNumber kD = new LoggedTunableNumber("Kicker/kD", 0.0);

    protected final TalonFX motor;
    private final TalonFXConfiguration motorConfig;

    private final StatusSignalCollection signals = new StatusSignalCollection();
    private final StatusSignal<Angle> positionSignal;
    private final StatusSignal<AngularVelocity> velocitySignal;
    private final StatusSignal<Voltage> voltageSignal;
    private final StatusSignal<Current> statorCurrentSignal;
    private final StatusSignal<Current> supplyCurrentSignal;
    private final StatusSignal<Temperature> temperatureSignal;
    private final StatusSignal<Double> closedLoopReferenceSignal;

    private final VoltageOut voltageOut = new VoltageOut(0.0);
    private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0);
    private final VelocityVoltage velocityVoltage = new VelocityVoltage(0);

    public KickerIOHardware() {
        motor = new TalonFX(motorId, PhoenixCANBus.canivore());
        motorConfig = new TalonFXConfiguration()
                .withMotorOutput(new MotorOutputConfigs()
                        .withNeutralMode(NeutralModeValue.Coast)
                        .withInverted(InvertedValue.CounterClockwise_Positive))
                .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(gearReduction))
                .withSlot0(new Slot0Configs()
                        .withKS(kS.get())
                        .withKV(kV.get())
                        .withKA(kA.get())
                        .withKP(kP.get())
                        .withKD(kD.get()))
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(Amps.of(120))
                        .withSupplyCurrentLimit(Amps.of(60))
                        .withSupplyCurrentLowerLimit(Amps.of(40)));
        PhoenixUtil.tryUntilOk(() -> motor.getConfigurator().apply(motorConfig));

        positionSignal = motor.getPosition();
        velocitySignal = motor.getVelocity();
        voltageSignal = motor.getMotorVoltage();
        statorCurrentSignal = motor.getStatorCurrent();
        supplyCurrentSignal = motor.getSupplyCurrent();
        temperatureSignal = motor.getDeviceTemp();
        closedLoopReferenceSignal = motor.getClosedLoopReference();

        signals.addSignals(
                positionSignal,
                velocitySignal,
                voltageSignal,
                statorCurrentSignal,
                supplyCurrentSignal,
                temperatureSignal,
                closedLoopReferenceSignal);

        signals.setUpdateFrequencyForAll(Constants.mainLoopFrequency);

        motor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(KickerInputs inputs) {
        signals.refreshAll();

        inputs.position = positionSignal.getValue();
        inputs.velocity = velocitySignal.getValue();
        inputs.voltage = voltageSignal.getValue();
        inputs.statorCurrent = statorCurrentSignal.getValue();
        inputs.supplyCurrent = supplyCurrentSignal.getValue();
        inputs.temperature = temperatureSignal.getValueAsDouble();

        LoggedTunableValue.ifChanged(
                0,
                () -> {
                    motorConfig.Slot0.kS = kS.get();
                    motorConfig.Slot0.kV = kV.get();
                    motorConfig.Slot0.kA = kA.get();
                    motorConfig.Slot0.kP = kP.get();
                    motorConfig.Slot0.kD = kD.get();
                    PhoenixUtil.tryUntilOk(() -> motor.getConfigurator().apply(motorConfig));
                },
                kS,
                kV,
                kA,
                kP,
                kD);
    }

    @Override
    public void setOutput(double percentOutput) {
        motor.setControl(dutyCycleOut.withOutput(percentOutput));
    }

    @Override
    public void setVelocity(AngularVelocity velocity) {
        motor.setControl(velocityVoltage.withVelocity(velocity));
    }

    @Override
    public void setVoltage(Voltage voltage) {
        motor.setControl(voltageOut.withOutput(voltage));
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }
}
