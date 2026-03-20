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

    protected final TalonFX leaderMotor;
    protected final TalonFX followerMotor;
    protected final TalonFXS extensionMotor;

    private final TalonFXConfiguration rollerConfig;
    private final TalonFXSConfiguration extensionConfig;

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

    private final StatusSignal<Angle> extensionPositionSignal;
    private final StatusSignal<AngularVelocity> extensionVelocitySignal;
    private final StatusSignal<Voltage> extensionVoltageSignal;
    private final StatusSignal<Current> extensionStatorCurrentSignal;
    private final StatusSignal<Current> extensionSupplyCurrentSignal;
    private final StatusSignal<Temperature> extensionTemperatureSignal;
    private final StatusSignal<Double> extensionClosedLoopReferenceSignal;

    private final DutyCycleOut rollerDutyCycleControlRequest = new DutyCycleOut(0.0);
    private final DutyCycleOut extensionDutyCycleControlRequest = new DutyCycleOut(0.0);
    private final MotionMagicVoltage extensionMotionMagicControlRequest = new MotionMagicVoltage(0.0).withSlot(0);
    private final PositionVoltage extensionPositionControlRequest = new PositionVoltage(0.0).withSlot(1);
    private final VoltageOut extensionVoltageControlRequest = new VoltageOut(0.0);

    public IntakeIOHardware() {
        leaderMotor = new TalonFX(leaderMotorId, PhoenixCANBus.canivore());
        followerMotor = new TalonFX(followerMotorId, PhoenixCANBus.canivore());

        extensionMotor = new TalonFXS(extensionMotorId, PhoenixCANBus.canivore());

        extensionConfig = new TalonFXSConfiguration()
                .withCommutation(new CommutationConfigs().withMotorArrangement(MotorArrangementValue.Minion_JST))
                .withMotorOutput(new MotorOutputConfigs()
                        .withNeutralMode(NeutralModeValue.Coast)
                        .withInverted(IntakeConstants.extensionMotorInvertedValue))
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
                        .withStatorCurrentLimit(Amps.of(20))
                        .withSupplyCurrentLimit(Amps.of(20))
                        .withSupplyCurrentLowerLimit(20));

        PhoenixUtil.tryUntilOk(() -> extensionMotor.getConfigurator().apply(extensionConfig));

        rollerConfig = new TalonFXConfiguration()
                .withMotorOutput(new MotorOutputConfigs()
                        .withNeutralMode(NeutralModeValue.Coast)
                        .withInverted(rollerMotorInvertedValue))
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(Amps.of(120))
                        .withSupplyCurrentLimit(Amps.of(40))
                        .withSupplyCurrentLowerLimit(40));

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

        extensionPositionSignal = extensionMotor.getPosition();
        extensionVelocitySignal = extensionMotor.getRotorVelocity();
        extensionVoltageSignal = extensionMotor.getMotorVoltage();
        extensionStatorCurrentSignal = extensionMotor.getStatorCurrent();
        extensionSupplyCurrentSignal = extensionMotor.getSupplyCurrent();
        extensionTemperatureSignal = extensionMotor.getDeviceTemp();
        extensionClosedLoopReferenceSignal = extensionMotor.getClosedLoopReference();

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
                extensionPositionSignal,
                extensionVelocitySignal,
                extensionVoltageSignal,
                extensionStatorCurrentSignal,
                extensionSupplyCurrentSignal,
                extensionTemperatureSignal,
                extensionClosedLoopReferenceSignal);

        signals.setUpdateFrequencyForAll(Constants.mainLoopFrequency);

        extensionMotor.optimizeBusUtilization();
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

        inputs.extensionPosition = Meters.of(
                extensionPositionSignal.getValue().in(Rotations) / extensionReduction * distancePerRotation.in(Meters));
        inputs.extensionVelocity =
                MetersPerSecond.of(extensionVelocitySignal.getValue().in(RotationsPerSecond)
                        / extensionReduction
                        * distancePerRotation.in(Meters));
        inputs.extensionVoltage = extensionVoltageSignal.getValue();
        inputs.extensionStatorCurrent = extensionStatorCurrentSignal.getValue();
        inputs.extensionSupplyCurrent = extensionSupplyCurrentSignal.getValue();
        inputs.extensionTemperature = extensionTemperatureSignal.getValueAsDouble();
        inputs.extensionClosedLoopReference = Meters.of(
                extensionClosedLoopReferenceSignal.getValue() / extensionReduction * distancePerRotation.in(Meters));

        LoggedTunableValue.ifChanged(
                0,
                () -> {
                    extensionConfig.MotionMagic.MotionMagicCruiseVelocity = motionMagicMaxVelocity.get();
                    extensionConfig.MotionMagic.MotionMagicAcceleration = motionMagicMaxAcceleration.get();
                    extensionConfig.MotionMagic.MotionMagicExpo_kV = extensionKV.get();
                    extensionConfig.MotionMagic.MotionMagicExpo_kA = extensionKA.get();
                    extensionConfig.Slot0.kG = extensionKG.get();
                    extensionConfig.Slot0.kS = extensionKS.get();
                    extensionConfig.Slot0.kV = extensionKV.get();
                    extensionConfig.Slot0.kA = extensionKA.get();
                    extensionConfig.Slot0.kP = motionMagicKP.get();
                    extensionConfig.Slot0.kD = motionMagicKD.get();
                    extensionConfig.Slot1.kG = extensionKG.get();
                    extensionConfig.Slot1.kS = extensionKS.get();
                    extensionConfig.Slot1.kV = extensionKV.get();
                    extensionConfig.Slot1.kA = extensionKA.get();
                    extensionConfig.Slot1.kP = extensionKP.get();
                    extensionConfig.Slot1.kD = extensionKD.get();
                    PhoenixUtil.tryUntilOk(
                            5, () -> extensionMotor.getConfigurator().apply(extensionConfig));
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
        extensionMotor.setControl(
                extensionDutyCycleControlRequest.withOutput(value).withIgnoreSoftwareLimits(!ignoreLimits));
    }

    @Override
    public void setExtensionPosition(Distance position) {
        var rotations = position.in(Meters) / distancePerRotation.in(Meters) * extensionReduction;
        if (useMotionMagic.get()) {
            extensionMotor.setControl(extensionMotionMagicControlRequest.withPosition(Rotations.of(rotations)));
        } else {
            extensionMotor.setControl(extensionPositionControlRequest.withPosition(Rotations.of(rotations)));
        }
    }

    @Override
    public void setExtensionNeutralMode(NeutralModeValue mode) {
        extensionMotor.setNeutralMode(mode);
    }

    @Override
    public void setExtensionVoltage(Voltage voltage) {
        extensionMotor.setControl(extensionVoltageControlRequest.withOutput(voltage));
    }

    @Override
    public void setExtensionEncoderPosition(Distance position) {
        extensionMotor.setPosition(
                Rotations.of(position.in(Meters) / distancePerRotation.in(Meters) * extensionReduction));
    }

    @Override
    public void stop() {
        leaderMotor.stopMotor();
        extensionMotor.stopMotor();
    }
}
