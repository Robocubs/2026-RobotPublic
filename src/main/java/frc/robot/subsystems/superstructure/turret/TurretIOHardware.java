// Copyright (c) 2026 FRC Team 1701 Robocubs
// http://github.com/Robocubs
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.superstructure.turret;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.StatusSignalCollection;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
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
import static frc.robot.subsystems.superstructure.turret.TurretConstants.*;

public class TurretIOHardware implements TurretIO {
    private static final LoggedTunableNumber kS = new LoggedTunableNumber("Turret/kS", 0.3033);
    private static final LoggedTunableNumber kV = new LoggedTunableNumber("Turret/kV", 0.62247 * 2 * Math.PI);
    private static final LoggedTunableNumber kA = new LoggedTunableNumber("Turret/kA", 0.018354 * 2 * Math.PI);
    private static final LoggedTunableNumber kP = new LoggedTunableNumber("Turret/kP", 150.0);
    private static final LoggedTunableNumber kD = new LoggedTunableNumber("Turret/kD", 5.0);

    protected final TalonFX motor;
    protected final CANcoder cancoder1;
    protected final CANcoder cancoder2;

    private final TalonFXConfiguration motorConfig;

    private final StatusSignalCollection signals = new StatusSignalCollection();
    private final StatusSignal<Angle> positionSignal;
    private final StatusSignal<AngularVelocity> velocitySignal;
    private final StatusSignal<Voltage> voltageSignal;
    private final StatusSignal<Current> statorCurrentSignal;
    private final StatusSignal<Current> supplyCurrentSignal;
    private final StatusSignal<Temperature> temperatureSignal;
    private final StatusSignal<Double> closedLoopReferenceSignal;
    private final StatusSignal<Angle> encoder1PositionSignal;
    private final StatusSignal<Angle> encoder2PositionSignal;

    private final PositionVoltage positionControlRequest = new PositionVoltage(0.0).withSlot(0);
    private final VoltageOut voltageOut = new VoltageOut(0.0);

    public TurretIOHardware() {
        motor = new TalonFX(motorId, PhoenixCANBus.canivore());
        cancoder1 = new CANcoder(cancoder1Id, PhoenixCANBus.canivore());
        cancoder2 = new CANcoder(cancoder2Id, PhoenixCANBus.canivore());

        motorConfig = new TalonFXConfiguration()
                .withMotorOutput(new MotorOutputConfigs()
                        .withNeutralMode(NeutralModeValue.Brake)
                        .withInverted(motorInvertedValue))
                .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(reduction))
                .withSlot0(new Slot0Configs()
                        .withKS(kS.get())
                        .withKV(kV.get())
                        .withKA(kA.get())
                        .withKP(kP.get())
                        .withKD(kD.get()))
                .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
                        .withForwardSoftLimitEnable(true)
                        .withForwardSoftLimitThreshold(maxAngle)
                        .withReverseSoftLimitEnable(true)
                        .withReverseSoftLimitThreshold(minAngle))
                .withCurrentLimits(
                        new CurrentLimitsConfigs() // TODO: Tune current limits
                                .withStatorCurrentLimit(Amps.of(80))
                                .withSupplyCurrentLimit(Amps.of(40))
                                .withSupplyCurrentLowerLimit(Amps.of(30)));

        PhoenixUtil.tryUntilOk(() -> motor.getConfigurator().apply(motorConfig));

        positionSignal = motor.getPosition();
        velocitySignal = motor.getVelocity();
        voltageSignal = motor.getMotorVoltage();
        statorCurrentSignal = motor.getStatorCurrent();
        supplyCurrentSignal = motor.getSupplyCurrent();
        temperatureSignal = motor.getDeviceTemp();
        closedLoopReferenceSignal = motor.getClosedLoopReference();
        encoder1PositionSignal = cancoder1.getPosition();
        encoder2PositionSignal = cancoder2.getPosition();

        signals.addSignals(
                positionSignal,
                velocitySignal,
                voltageSignal,
                statorCurrentSignal,
                supplyCurrentSignal,
                temperatureSignal,
                closedLoopReferenceSignal,
                encoder1PositionSignal,
                encoder2PositionSignal);

        signals.setUpdateFrequencyForAll(Constants.mainLoopFrequency);

        motor.optimizeBusUtilization();
        cancoder1.optimizeBusUtilization();
        cancoder2.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(TurretInputs inputs) {
        signals.refreshAll();

        inputs.timestamp = PhoenixUtil.getFPGATimestamp(positionSignal);
        inputs.position = positionSignal.getValue();
        inputs.velocity = velocitySignal.getValue();
        inputs.voltage = voltageSignal.getValue();
        inputs.statorCurrent = statorCurrentSignal.getValue();
        inputs.supplyCurrent = supplyCurrentSignal.getValue();
        inputs.temperature = temperatureSignal.getValueAsDouble();
        inputs.closedLoopReference = Rotations.of(closedLoopReferenceSignal.getValue());
        inputs.encoder1Connected = encoder1PositionSignal.getStatus() == StatusCode.OK;
        inputs.encoder2Connected = encoder2PositionSignal.getStatus() == StatusCode.OK;
        inputs.encoder1Position = encoder1PositionSignal.getValue();
        inputs.encoder2Position = encoder2PositionSignal.getValue();

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
    public void setPosition(Angle angle) {
        motor.setControl(positionControlRequest.withPosition(angle).withVelocity(0));
    }

    @Override
    public void setPosition(Angle angle, AngularVelocity velocity) {
        motor.setControl(positionControlRequest.withPosition(angle).withVelocity(velocity));
    }

    @Override
    public void setVoltage(Voltage voltage) {
        motor.setControl(voltageOut.withOutput(voltage.in(Volts)));
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }

    @Override
    public void resetPosition(Angle angle) {
        motor.setPosition(angle);
    }
}
