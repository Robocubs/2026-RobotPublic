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
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
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
    private static final LoggedTunableNumber motionMagicKP = new LoggedTunableNumber("Intake/MotionMagicKP", 0.0);
    private static final LoggedTunableNumber motionMagicKD = new LoggedTunableNumber("Intake/MotionMagicKD", 0.0);

    private static final LoggedTunableNumber extensionKG = new LoggedTunableNumber("Intake/ExtensionKG", 0.0);
    private static final LoggedTunableNumber extensionKS = new LoggedTunableNumber("Intake/ExtensionKS", 0.0);
    private static final LoggedTunableNumber extensionKV = new LoggedTunableNumber("Intake/ExtensionKV", 0.0);
    private static final LoggedTunableNumber extensionKA = new LoggedTunableNumber("Intake/ExtensionKA", 0.0);
    private static final LoggedTunableNumber extensionKP = new LoggedTunableNumber("Intake/ExtensionKP", 0.0);
    private static final LoggedTunableNumber extensionKD = new LoggedTunableNumber("Intake/ExtensionKD", 0.0);

    private final StatusSignalCollection signals = new StatusSignalCollection();
    private final StatusSignal<Angle> rollerPositionSignal;
    private final StatusSignal<AngularVelocity> rollerVelocitySignal;
    private final StatusSignal<Voltage> rollerVoltageSignal;
    private final StatusSignal<Current> rollerStatorCurrentSignal;
    private final StatusSignal<Current> rollerSupplyCurrentSignal;
    private final StatusSignal<Temperature> rollerTemperatureSignal;
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

    protected final TalonFX rollerMotor;
    protected final TalonFXS extensionMotor;
    private final TalonFXSConfiguration extensionConfig;

    public IntakeIOHardware() {
        rollerMotor = new TalonFX(rollerMotorId, PhoenixCANBus.canivore());
        extensionMotor = new TalonFXS(extensionMotorId, PhoenixCANBus.canivore());

        extensionConfig = new TalonFXSConfiguration()
                .withCommutation(new CommutationConfigs().withMotorArrangement(MotorArrangementValue.Minion_JST))
                .withMotorOutput(new MotorOutputConfigs()
                        .withNeutralMode(NeutralModeValue.Coast)
                        .withInverted(IntakeConstants.motorInvertedValue))
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
                        .withStatorCurrentLimit(Amps.of(40))
                        .withSupplyCurrentLimit(Amps.of(40))
                        .withSupplyCurrentLowerLimit(30));

        PhoenixUtil.tryUntilOk(5, () -> extensionMotor.getConfigurator().apply(extensionConfig));

        TalonFXConfiguration rollerConfig = new TalonFXConfiguration()
                .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast))
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(Amps.of(120))
                        .withSupplyCurrentLimit(Amps.of(60))
                        .withSupplyCurrentLowerLimit(40));
        PhoenixUtil.tryUntilOk(5, () -> rollerMotor.getConfigurator().apply(rollerConfig));

        rollerPositionSignal = rollerMotor.getPosition();
        rollerVelocitySignal = rollerMotor.getVelocity();
        rollerVoltageSignal = rollerMotor.getMotorVoltage();
        rollerStatorCurrentSignal = rollerMotor.getStatorCurrent();
        rollerSupplyCurrentSignal = rollerMotor.getSupplyCurrent();
        rollerTemperatureSignal = rollerMotor.getDeviceTemp();
        extensionPositionSignal = extensionMotor.getPosition();
        extensionVelocitySignal = extensionMotor.getRotorVelocity();
        extensionVoltageSignal = extensionMotor.getMotorVoltage();
        extensionStatorCurrentSignal = extensionMotor.getStatorCurrent();
        extensionSupplyCurrentSignal = extensionMotor.getSupplyCurrent();
        extensionTemperatureSignal = extensionMotor.getDeviceTemp();
        extensionClosedLoopReferenceSignal = extensionMotor.getClosedLoopReference();

        signals.addSignals(
                rollerPositionSignal,
                rollerVelocitySignal,
                rollerVoltageSignal,
                rollerStatorCurrentSignal,
                rollerSupplyCurrentSignal,
                rollerTemperatureSignal,
                extensionPositionSignal,
                extensionVelocitySignal,
                extensionVoltageSignal,
                extensionStatorCurrentSignal,
                extensionSupplyCurrentSignal,
                extensionTemperatureSignal,
                extensionClosedLoopReferenceSignal);

        signals.setUpdateFrequencyForAll(Constants.mainLoopFrequency);

        extensionMotor.optimizeBusUtilization();
        rollerMotor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(IntakeInputs inputs) {
        signals.refreshAll();

        inputs.rollerPosition = rollerPositionSignal.getValue();
        inputs.rollerVelocity = rollerVelocitySignal.getValue();
        inputs.rollerVoltage = rollerVoltageSignal.getValue();
        inputs.rollerStatorCurrent = rollerStatorCurrentSignal.getValue();
        inputs.rollerSupplyCurrent = rollerSupplyCurrentSignal.getValue();
        inputs.rollerTemperature = rollerTemperatureSignal.getValueAsDouble();

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
        rollerMotor.setControl(rollerDutyCycleControlRequest.withOutput(value));
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
        rollerMotor.stopMotor();
        extensionMotor.stopMotor();
    }
}
