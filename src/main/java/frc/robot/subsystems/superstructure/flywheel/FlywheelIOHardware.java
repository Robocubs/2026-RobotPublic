// Copyright (c) 2026 FRC Team 1701 Robocubs
// http://github.com/Robocubs
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.superstructure.flywheel;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.StatusSignalCollection;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
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
import static frc.robot.subsystems.superstructure.flywheel.FlywheelConstants.*;

public class FlywheelIOHardware implements FlywheelIO {
    private static final LoggedTunableNumber kS = new LoggedTunableNumber("Flywheel/kS", 0.17591);
    private static final LoggedTunableNumber kV =
            new LoggedTunableNumber("Flywheel/kV", Units.rotationsToRadians(0.018674));
    private static final LoggedTunableNumber kA =
            new LoggedTunableNumber("Flywheel/kA", Units.rotationsToRadians(0.00088465));
    private static final LoggedTunableNumber kP = new LoggedTunableNumber("Flywheel/kP", 0.02);
    private static final LoggedTunableNumber kD = new LoggedTunableNumber("Flywheel/kD", 0.0);

    protected final TalonFX leaderMotor;
    protected final TalonFX followerMotor;

    private final TalonFXConfiguration motorConfig;

    private final StatusSignalCollection signals = new StatusSignalCollection();
    private final StatusSignal<Angle> leaderPositionSignal;
    private final StatusSignal<AngularVelocity> leaderVelocitySignal;
    private final StatusSignal<Voltage> leaderVoltageSignal;
    private final StatusSignal<Current> leaderStatorCurrentSignal;
    private final StatusSignal<Current> leaderSupplyCurrentSignal;
    private final StatusSignal<Temperature> leaderTemperatureSignal;
    private final StatusSignal<Angle> followerPositionSignal;
    private final StatusSignal<AngularVelocity> followerVelocitySignal;
    private final StatusSignal<Voltage> followerVoltageSignal;
    private final StatusSignal<Current> followerStatorCurrentSignal;
    private final StatusSignal<Current> followerSupplyCurrentSignal;
    private final StatusSignal<Temperature> followerTemperatureSignal;
    private final StatusSignal<Double> closedLoopReferenceSignal;

    private final VelocityVoltage velocityControlRequest = new VelocityVoltage(0.0).withSlot(0);
    private final VoltageOut voltageOut = new VoltageOut(0.0);

    public FlywheelIOHardware() {
        this.leaderMotor = new TalonFX(leaderId, PhoenixCANBus.canivore());
        this.followerMotor = new TalonFX(followerId, PhoenixCANBus.canivore());

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
                        .withStatorCurrentLimit(Amps.of(80))
                        .withSupplyCurrentLimit(Amps.of(60))
                        .withSupplyCurrentLowerLimit(Amps.of(40)));

        PhoenixUtil.tryUntilOk(() -> leaderMotor.getConfigurator().apply(motorConfig));
        PhoenixUtil.tryUntilOk(() -> followerMotor.getConfigurator().apply(motorConfig));
        PhoenixUtil.tryUntilOk(() -> followerMotor.setControl(new Follower(leaderId, MotorAlignmentValue.Aligned)));

        leaderPositionSignal = leaderMotor.getPosition();
        leaderVelocitySignal = leaderMotor.getVelocity();
        leaderVoltageSignal = leaderMotor.getMotorVoltage();
        leaderStatorCurrentSignal = leaderMotor.getStatorCurrent();
        leaderSupplyCurrentSignal = leaderMotor.getSupplyCurrent();
        leaderTemperatureSignal = leaderMotor.getDeviceTemp();
        followerPositionSignal = followerMotor.getPosition();
        followerVelocitySignal = followerMotor.getVelocity();
        followerVoltageSignal = followerMotor.getMotorVoltage();
        followerStatorCurrentSignal = followerMotor.getStatorCurrent();
        followerSupplyCurrentSignal = followerMotor.getSupplyCurrent();
        followerTemperatureSignal = followerMotor.getDeviceTemp();
        closedLoopReferenceSignal = leaderMotor.getClosedLoopReference();

        signals.addSignals(
                leaderPositionSignal,
                leaderVelocitySignal,
                leaderVoltageSignal,
                leaderStatorCurrentSignal,
                leaderSupplyCurrentSignal,
                leaderTemperatureSignal,
                followerPositionSignal,
                followerVelocitySignal,
                followerVoltageSignal,
                followerStatorCurrentSignal,
                followerSupplyCurrentSignal,
                followerTemperatureSignal,
                closedLoopReferenceSignal);

        signals.setUpdateFrequencyForAll(Constants.mainLoopFrequency);

        leaderMotor.optimizeBusUtilization();
        followerMotor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(FlywheelInputs inputs) {
        signals.refreshAll();

        inputs.leaderPosition = leaderPositionSignal.getValue();
        inputs.leaderVelocity = leaderVelocitySignal.getValue();
        inputs.leaderVoltage = leaderVoltageSignal.getValue();
        inputs.leaderStatorCurrent = leaderStatorCurrentSignal.getValue();
        inputs.leaderSupplyCurrent = leaderSupplyCurrentSignal.getValue();
        inputs.leaderTemperature = leaderTemperatureSignal.getValueAsDouble();

        inputs.followerPosition = followerPositionSignal.getValue();
        inputs.followerVelocity = followerVelocitySignal.getValue();
        inputs.followerVoltage = followerVoltageSignal.getValue();
        inputs.followerStatorCurrent = followerStatorCurrentSignal.getValue();
        inputs.followerSupplyCurrent = followerSupplyCurrentSignal.getValue();
        inputs.followerTemperature = followerTemperatureSignal.getValueAsDouble();

        inputs.closedLoopReference = closedLoopReferenceSignal.getValue();

        LoggedTunableValue.ifChanged(
                0,
                () -> {
                    motorConfig.Slot0.kS = kS.get();
                    motorConfig.Slot0.kV = kV.get();
                    motorConfig.Slot0.kA = kA.get();
                    motorConfig.Slot0.kP = kP.get();
                    motorConfig.Slot0.kD = kD.get();
                    PhoenixUtil.tryUntilOk(() -> leaderMotor.getConfigurator().apply(motorConfig));
                    PhoenixUtil.tryUntilOk(() -> followerMotor.getConfigurator().apply(motorConfig));
                },
                kS,
                kV,
                kA,
                kP,
                kD);
    }

    @Override
    public void setVelocity(AngularVelocity velocity) {
        leaderMotor.setControl(velocityControlRequest.withVelocity(velocity));
    }

    @Override
    public void setVoltage(Voltage voltage) {
        leaderMotor.setControl(voltageOut.withOutput(voltage));
    }

    @Override
    public void stop() {
        leaderMotor.stopMotor();
    }
}
