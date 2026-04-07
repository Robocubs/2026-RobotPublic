// Copyright (c) 2026 FRC Team 1701 Robocubs
// http://github.com/Robocubs
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.StatusSignalCollection;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.CommutationConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.PhoenixCANBus;
import frc.robot.util.PhoenixUtil;
import frc.robot.util.tuning.LoggedTunableBoolean;
import frc.robot.util.tuning.LoggedTunableNumber;
import frc.robot.util.tuning.LoggedTunableValue;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.intake.IntakeConstants.*;

public class IntakeIOHardware implements IntakeIO {
    private static final LoggedTunableBoolean useMotionMagic = new LoggedTunableBoolean("Intake/UseMotionMagic", true);
    private static final LoggedTunableNumber motionMagicMaxVelocity = new LoggedTunableNumber(
            "Intake/MotionMagicMaxVelocity",
            maximumLinearVelocity.in(MetersPerSecond) / distancePerRotation.in(Meters) * extensionReduction);
    private static final LoggedTunableNumber motionMagicMaxAcceleration = new LoggedTunableNumber(
            "Intake/MotionMagicMaxAcceleration",
            maximumLinearVelocity.in(MetersPerSecond) / 0.2 / distancePerRotation.in(Meters) * extensionReduction);
    private static final LoggedTunableNumber motionMagicKP =
            new LoggedTunableNumber("Intake/MotionMagicKP", Constants.currentMode == Mode.SIM ? 5.0 : 1.0);
    private static final LoggedTunableNumber motionMagicKD = new LoggedTunableNumber("Intake/MotionMagicKD", 0.0);

    private static final LoggedTunableNumber extensionKG = new LoggedTunableNumber("Intake/ExtensionKG", 0.0);
    private static final LoggedTunableNumber extensionKS = new LoggedTunableNumber("Intake/ExtensionKS", 0.0);
    private static final LoggedTunableNumber extensionKV = new LoggedTunableNumber("Intake/ExtensionKV", 0.0);
    private static final LoggedTunableNumber extensionKA = new LoggedTunableNumber("Intake/ExtensionKA", 0.0);
    private static final LoggedTunableNumber extensionKP =
            new LoggedTunableNumber("Intake/ExtensionKP", Constants.currentMode == Mode.SIM ? 1.0 : 1.0);
    private static final LoggedTunableNumber extensionKD = new LoggedTunableNumber("Intake/ExtensionKD", 0.0);

    private final DigitalInput limitSwitch1 = new DigitalInput(0);
    private final DigitalInput limitSwitch2 = new DigitalInput(6);

    protected final TalonFX leaderMotor;
    protected final TalonFX followerMotor;
    protected final TalonFXS extensionMotor1;
    protected final TalonFXS extensionMotor2;

    private final TalonFXConfiguration rollerConfig;
    private final TalonFXSConfiguration extension1Config;
    private final TalonFXSConfiguration extension2Config;

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

    private final StatusSignal<Angle> extension1PositionSignal;
    private final StatusSignal<AngularVelocity> extension1VelocitySignal;
    private final StatusSignal<Voltage> extension1VoltageSignal;
    private final StatusSignal<Current> extension1StatorCurrentSignal;
    private final StatusSignal<Current> extension1SupplyCurrentSignal;
    private final StatusSignal<Temperature> extension1TemperatureSignal;
    private final StatusSignal<Double> extension1ClosedLoopReferenceSignal;

    private final StatusSignal<Angle> extension2PositionSignal;
    private final StatusSignal<AngularVelocity> extension2VelocitySignal;
    private final StatusSignal<Voltage> extension2VoltageSignal;
    private final StatusSignal<Current> extension2StatorCurrentSignal;
    private final StatusSignal<Current> extension2SupplyCurrentSignal;
    private final StatusSignal<Temperature> extension2TemperatureSignal;
    private final StatusSignal<Double> extension2ClosedLoopReferenceSignal;

    private final DutyCycleOut rollerDutyCycleControlRequest = new DutyCycleOut(0.0).withEnableFOC(false);
    private final DutyCycleOut extensionDutyCycleControlRequest = new DutyCycleOut(0.0);
    private final MotionMagicVoltage extensionMotionMagicControlRequest = new MotionMagicVoltage(0.0).withSlot(0);
    private final PositionVoltage extensionPositionControlRequest = new PositionVoltage(0.0).withSlot(1);
    private final VoltageOut extensionVoltageControlRequest = new VoltageOut(0.0);

    public IntakeIOHardware() {
        leaderMotor = new TalonFX(leaderMotorId, PhoenixCANBus.canivore());
        followerMotor = new TalonFX(followerMotorId, PhoenixCANBus.canivore());

        extensionMotor1 = new TalonFXS(extensionMotor1Id, PhoenixCANBus.canivore());
        extensionMotor2 = new TalonFXS(extensionMotor2Id, PhoenixCANBus.canivore());

        extension1Config = new TalonFXSConfiguration()
                .withCommutation(new CommutationConfigs().withMotorArrangement(MotorArrangementValue.Minion_JST))
                .withMotorOutput(new MotorOutputConfigs()
                        .withNeutralMode(NeutralModeValue.Coast)
                        .withInverted(IntakeConstants.extension1MotorInvertedValue))
                .withClosedLoopGeneral(new ClosedLoopGeneralConfigs().withContinuousWrap(false))
                .withMotionMagic(new MotionMagicConfigs()
                        .withMotionMagicCruiseVelocity(motionMagicMaxVelocity.get())
                        .withMotionMagicAcceleration(motionMagicMaxAcceleration.get())
                        .withMotionMagicExpo_kV(extensionKV.get())
                        .withMotionMagicExpo_kA(extensionKA.get()))
                .withSlot0(new Slot0Configs()
                        .withKG(extensionKG.get())
                        .withKS(extensionKS.get())
                        .withKV(extensionKV.get())
                        .withKA(extensionKA.get())
                        .withKP(motionMagicKP.get())
                        .withKD(motionMagicKD.get()))
                .withSlot1(new Slot1Configs()
                        .withKG(extensionKG.get())
                        .withKS(extensionKS.get())
                        .withKV(extensionKV.get())
                        .withKA(extensionKA.get())
                        .withKP(extensionKP.get())
                        .withKD(extensionKD.get()))
                .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
                        .withForwardSoftLimitEnable(true)
                        .withForwardSoftLimitThreshold(
                                maximumExtension.in(Meters) / distancePerRotation.in(Meters) * extensionReduction)
                        .withReverseSoftLimitEnable(true)
                        .withReverseSoftLimitThreshold(
                                minimumExtension.in(Meters) / distancePerRotation.in(Meters) * extensionReduction))
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(Amps.of(30))
                        .withSupplyCurrentLimit(Amps.of(30))
                        .withSupplyCurrentLowerLimit(30));

        extension2Config = new TalonFXSConfiguration()
                .withCommutation(new CommutationConfigs().withMotorArrangement(MotorArrangementValue.Minion_JST))
                .withMotorOutput(new MotorOutputConfigs()
                        .withNeutralMode(NeutralModeValue.Coast)
                        .withInverted(IntakeConstants.extension2MotorInvertedValue))
                .withClosedLoopGeneral(new ClosedLoopGeneralConfigs().withContinuousWrap(false))
                .withMotionMagic(new MotionMagicConfigs()
                        .withMotionMagicCruiseVelocity(motionMagicMaxVelocity.get())
                        .withMotionMagicAcceleration(motionMagicMaxAcceleration.get())
                        .withMotionMagicExpo_kV(extensionKV.get())
                        .withMotionMagicExpo_kA(extensionKA.get()))
                .withSlot0(new Slot0Configs()
                        .withKG(extensionKG.get())
                        .withKS(extensionKS.get())
                        .withKV(extensionKV.get())
                        .withKA(extensionKA.get())
                        .withKP(motionMagicKP.get())
                        .withKD(motionMagicKD.get()))
                .withSlot1(new Slot1Configs()
                        .withKG(extensionKG.get())
                        .withKS(extensionKS.get())
                        .withKV(extensionKV.get())
                        .withKA(extensionKA.get())
                        .withKP(extensionKP.get())
                        .withKD(extensionKD.get()))
                .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
                        .withForwardSoftLimitEnable(true)
                        .withForwardSoftLimitThreshold(
                                maximumExtension.in(Meters) / distancePerRotation.in(Meters) * extensionReduction)
                        .withReverseSoftLimitEnable(true)
                        .withReverseSoftLimitThreshold(
                                minimumExtension.in(Meters) / distancePerRotation.in(Meters) * extensionReduction))
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(Amps.of(30))
                        .withSupplyCurrentLimit(Amps.of(30))
                        .withSupplyCurrentLowerLimit(30));

        PhoenixUtil.tryUntilOk(() -> extensionMotor1.getConfigurator().apply(extension1Config));
        PhoenixUtil.tryUntilOk(() -> extensionMotor2.getConfigurator().apply(extension2Config));

        rollerConfig = new TalonFXConfiguration()
                .withMotorOutput(new MotorOutputConfigs()
                        .withNeutralMode(NeutralModeValue.Coast)
                        .withInverted(rollerMotorInvertedValue))
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(Amps.of(120))
                        .withSupplyCurrentLimit(Amps.of(40))
                        .withSupplyCurrentLowerLimit(30));

        PhoenixUtil.tryUntilOk(() -> leaderMotor.getConfigurator().apply(rollerConfig));
        PhoenixUtil.tryUntilOk(() -> followerMotor.getConfigurator().apply(rollerConfig));
        PhoenixUtil.tryUntilOk(
                () -> followerMotor.setControl(new Follower(leaderMotorId, MotorAlignmentValue.Opposed)));

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

        extension1PositionSignal = extensionMotor1.getPosition();
        extension1VelocitySignal = extensionMotor1.getRotorVelocity();
        extension1VoltageSignal = extensionMotor1.getMotorVoltage();
        extension1StatorCurrentSignal = extensionMotor1.getStatorCurrent();
        extension1SupplyCurrentSignal = extensionMotor1.getSupplyCurrent();
        extension1TemperatureSignal = extensionMotor1.getDeviceTemp();
        extension1ClosedLoopReferenceSignal = extensionMotor1.getClosedLoopReference();

        extension2PositionSignal = extensionMotor2.getPosition();
        extension2VelocitySignal = extensionMotor2.getRotorVelocity();
        extension2VoltageSignal = extensionMotor2.getMotorVoltage();
        extension2StatorCurrentSignal = extensionMotor2.getStatorCurrent();
        extension2SupplyCurrentSignal = extensionMotor2.getSupplyCurrent();
        extension2TemperatureSignal = extensionMotor2.getDeviceTemp();
        extension2ClosedLoopReferenceSignal = extensionMotor2.getClosedLoopReference();

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
                extension1PositionSignal,
                extension1VelocitySignal,
                extension1VoltageSignal,
                extension1StatorCurrentSignal,
                extension1SupplyCurrentSignal,
                extension1TemperatureSignal,
                extension1ClosedLoopReferenceSignal,
                extension2PositionSignal,
                extension2VelocitySignal,
                extension2VoltageSignal,
                extension2StatorCurrentSignal,
                extension2SupplyCurrentSignal,
                extension2TemperatureSignal,
                extension2ClosedLoopReferenceSignal);

        signals.setUpdateFrequencyForAll(Constants.mainLoopFrequency);

        extensionMotor1.optimizeBusUtilization();
        extensionMotor2.optimizeBusUtilization();
        leaderMotor.optimizeBusUtilization();
        followerMotor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(IntakeInputs inputs) {
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

        inputs.extension1Position =
                Meters.of(extension1PositionSignal.getValue().in(Rotations)
                        / extensionReduction
                        * distancePerRotation.in(Meters));
        inputs.extension1Velocity =
                MetersPerSecond.of(extension1VelocitySignal.getValue().in(RotationsPerSecond)
                        / extensionReduction
                        * distancePerRotation.in(Meters));
        inputs.extension1Voltage = extension1VoltageSignal.getValue();
        inputs.extension1StatorCurrent = extension1StatorCurrentSignal.getValue();
        inputs.extension1SupplyCurrent = extension1SupplyCurrentSignal.getValue();
        inputs.extension1Temperature = extension1TemperatureSignal.getValueAsDouble();
        inputs.extension1ClosedLoopReference = Meters.of(
                extension1ClosedLoopReferenceSignal.getValue() / extensionReduction * distancePerRotation.in(Meters));

        inputs.extension2Position =
                Meters.of(extension2PositionSignal.getValue().in(Rotations)
                        / extensionReduction
                        * distancePerRotation.in(Meters));
        inputs.extension2Velocity =
                MetersPerSecond.of(extension2VelocitySignal.getValue().in(RotationsPerSecond)
                        / extensionReduction
                        * distancePerRotation.in(Meters));
        inputs.extension2Voltage = extension2VoltageSignal.getValue();
        inputs.extension2StatorCurrent = extension2StatorCurrentSignal.getValue();
        inputs.extension2SupplyCurrent = extension2SupplyCurrentSignal.getValue();
        inputs.extension2Temperature = extension2TemperatureSignal.getValueAsDouble();
        inputs.extension2ClosedLoopReference = Meters.of(
                extension2ClosedLoopReferenceSignal.getValue() / extensionReduction * distancePerRotation.in(Meters));

        inputs.limitSwitch1 = !limitSwitch1.get();
        inputs.limitSwitch2 = !limitSwitch2.get();

        LoggedTunableValue.ifChanged(
                0,
                () -> {
                    extension1Config.MotionMagic.MotionMagicCruiseVelocity = motionMagicMaxVelocity.get();
                    extension1Config.MotionMagic.MotionMagicAcceleration = motionMagicMaxAcceleration.get();
                    extension1Config.MotionMagic.MotionMagicExpo_kV = extensionKV.get();
                    extension1Config.MotionMagic.MotionMagicExpo_kA = extensionKA.get();
                    extension1Config.Slot0.kG = extensionKG.get();
                    extension1Config.Slot0.kS = extensionKS.get();
                    extension1Config.Slot0.kV = extensionKV.get();
                    extension1Config.Slot0.kA = extensionKA.get();
                    extension1Config.Slot0.kP = motionMagicKP.get();
                    extension1Config.Slot0.kD = motionMagicKD.get();
                    extension1Config.Slot1.kG = extensionKG.get();
                    extension1Config.Slot1.kS = extensionKS.get();
                    extension1Config.Slot1.kV = extensionKV.get();
                    extension1Config.Slot1.kA = extensionKA.get();
                    extension1Config.Slot1.kP = extensionKP.get();
                    extension1Config.Slot1.kD = extensionKD.get();

                    extension2Config.MotionMagic.MotionMagicCruiseVelocity = motionMagicMaxVelocity.get();
                    extension2Config.MotionMagic.MotionMagicAcceleration = motionMagicMaxAcceleration.get();
                    extension2Config.MotionMagic.MotionMagicExpo_kV = extensionKV.get();
                    extension2Config.MotionMagic.MotionMagicExpo_kA = extensionKA.get();
                    extension2Config.Slot0.kG = extensionKG.get();
                    extension2Config.Slot0.kS = extensionKS.get();
                    extension2Config.Slot0.kV = extensionKV.get();
                    extension2Config.Slot0.kA = extensionKA.get();
                    extension2Config.Slot0.kP = motionMagicKP.get();
                    extension2Config.Slot0.kD = motionMagicKD.get();
                    extension2Config.Slot1.kG = extensionKG.get();
                    extension2Config.Slot1.kS = extensionKS.get();
                    extension2Config.Slot1.kV = extensionKV.get();
                    extension2Config.Slot1.kA = extensionKA.get();
                    extension2Config.Slot1.kP = extensionKP.get();
                    extension2Config.Slot1.kD = extensionKD.get();

                    PhoenixUtil.tryUntilOk(
                            5, () -> extensionMotor1.getConfigurator().apply(extension1Config));
                    PhoenixUtil.tryUntilOk(
                            5, () -> extensionMotor2.getConfigurator().apply(extension2Config));
                },
                motionMagicMaxVelocity,
                motionMagicMaxAcceleration,
                motionMagicKP,
                motionMagicKD,
                extensionKG,
                extensionKS,
                extensionKV,
                extensionKA,
                extensionKP,
                extensionKD);
    }

    @Override
    public void setRollerOutput(double value) {
        leaderMotor.setControl(rollerDutyCycleControlRequest.withOutput(value));
    }

    @Override
    public void setExtensionOutput(double value) {
        setExtensionOutput(value, false);
    }

    @Override
    public void setExtensionOutput(double value, boolean ignoreLimits) {
        extensionMotor1.setControl(
                extensionDutyCycleControlRequest.withOutput(value).withIgnoreSoftwareLimits(ignoreLimits));
        extensionMotor2.setControl(
                extensionDutyCycleControlRequest.withOutput(value).withIgnoreSoftwareLimits(ignoreLimits));
    }

    @Override
    public void setExtensionOutput(double value1, double value2, boolean ignoreLimits) {
        extensionMotor1.setControl(
                extensionDutyCycleControlRequest.withOutput(value1).withIgnoreSoftwareLimits(ignoreLimits));
        extensionMotor2.setControl(
                extensionDutyCycleControlRequest.withOutput(value2).withIgnoreSoftwareLimits(ignoreLimits));
    }

    @Override
    public void setExtensionPosition(Distance position) {
        var rotations = position.in(Meters) / distancePerRotation.in(Meters) * extensionReduction;
        if (useMotionMagic.get()) {
            extensionMotor1.setControl(extensionMotionMagicControlRequest.withPosition(rotations));
            extensionMotor2.setControl(extensionMotionMagicControlRequest.withPosition(rotations));
        } else {
            extensionMotor1.setControl(extensionPositionControlRequest.withPosition(rotations));
            extensionMotor2.setControl(extensionPositionControlRequest.withPosition(rotations));
        }
    }

    @Override
    public void setExtensionNeutralMode(NeutralModeValue mode) {
        extensionMotor1.setNeutralMode(mode);
        extensionMotor2.setNeutralMode(mode);
    }

    @Override
    public void setExtensionVoltage(Voltage voltage) {
        extensionMotor1.setControl(extensionVoltageControlRequest.withOutput(voltage));
        extensionMotor2.setControl(extensionVoltageControlRequest.withOutput(voltage));
    }

    @Override
    public void setExtensionEncoderPosition(Distance position) {
        extensionMotor1.setPosition(
                Rotations.of(position.in(Meters) / distancePerRotation.in(Meters) * extensionReduction));
        extensionMotor2.setPosition(
                Rotations.of(position.in(Meters) / distancePerRotation.in(Meters) * extensionReduction));
    }

    @Override
    public void stop() {
        leaderMotor.stopMotor();
        extensionMotor1.stopMotor();
        extensionMotor2.stopMotor();
    }
}
