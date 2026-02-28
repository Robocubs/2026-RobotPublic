// Copyright (c) 2026 FRC Team 1701 Robocubs
// http://github.com/Robocubs
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.StatusSignalCollection;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
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
    private static final LoggedTunableNumber motionMagicMaxVelocity =
            new LoggedTunableNumber("Intake/MotionMagicMaxVelocity", maximumVelocity.in(RotationsPerSecond));
    private static final LoggedTunableNumber motionMagicMaxAcceleration = new LoggedTunableNumber(
            "Intake/MotionMagicMaxAcceleration", maximumAcceleration.in(RotationsPerSecondPerSecond));
    private static final LoggedTunableNumber motionMagicMaxJerk = new LoggedTunableNumber(
            "Intake/MotionMagicMaxJerk", maximumJerk.in(RotationsPerSecondPerSecond.per(Second)));
    private static final LoggedTunableNumber motionMagicKP = new LoggedTunableNumber("Intake/MotionMagicKP", 0.0);
    private static final LoggedTunableNumber motionMagicKD = new LoggedTunableNumber("Intake/MotionMagicKD", 0.0);

    private static final LoggedTunableNumber pivotKG = new LoggedTunableNumber("Intake/PivotKG", 0.0);
    private static final LoggedTunableNumber pivotKS = new LoggedTunableNumber("Intake/PivotKS", 0.0);
    private static final LoggedTunableNumber pivotKV = new LoggedTunableNumber("Intake/PivotKV", 0.0);
    private static final LoggedTunableNumber pivotKA = new LoggedTunableNumber("Intake/PivotKA", 0.0);
    private static final LoggedTunableNumber pivotKP = new LoggedTunableNumber("Intake/PivotKP", 0.0);
    private static final LoggedTunableNumber pivotKD = new LoggedTunableNumber("Intake/PivotKD", 0.0);

    private final StatusSignalCollection signals = new StatusSignalCollection();
    private final StatusSignal<Angle> rollerPositionSignal;
    private final StatusSignal<AngularVelocity> rollerVelocitySignal;
    private final StatusSignal<Voltage> rollerVoltageSignal;
    private final StatusSignal<Current> rollerStatorCurrentSignal;
    private final StatusSignal<Current> rollerSupplyCurrentSignal;
    private final StatusSignal<Temperature> rollerTemperatureSignal;
    private final StatusSignal<Angle> pivotAngleSignal;
    private final StatusSignal<Angle> pivotPositionSignal;
    private final StatusSignal<AngularVelocity> pivotVelocitySignal;
    private final StatusSignal<Voltage> pivotVoltageSignal;
    private final StatusSignal<Current> pivotStatorCurrentSignal;
    private final StatusSignal<Current> pivotSupplyCurrentSignal;
    private final StatusSignal<Temperature> pivotTemperatureSignal;
    private final StatusSignal<Double> pivotClosedLoopReferenceSignal;

    private final DutyCycleOut rollerDutyCycleControlRequest = new DutyCycleOut(0.0);
    private final DutyCycleOut pivotDutyCycleControlRequest = new DutyCycleOut(0.0);
    private final MotionMagicVoltage pivotMotionMagicControlRequest = new MotionMagicVoltage(0.0).withSlot(0);
    private final PositionVoltage pivotPositionControlRequest = new PositionVoltage(0.0).withSlot(1);

    protected final TalonFX rollerMotor;
    protected final TalonFX pivotMotor;
    protected final CANcoder cancoder;
    private final CANcoderConfiguration cancoderConfig;
    private final TalonFXConfiguration motorConfig;

    public IntakeIOHardware() {
        rollerMotor = new TalonFX(rollerMotorId, PhoenixCANBus.canivore());
        pivotMotor = new TalonFX(pivotMotorId, PhoenixCANBus.canivore());

        var feedbackConfigs = new FeedbackConfigs();
        if (feedbackSensorSource == FeedbackSensorSourceValue.FusedCANcoder
                || feedbackSensorSource == FeedbackSensorSourceValue.SyncCANcoder
                || feedbackSensorSource == FeedbackSensorSourceValue.RemoteCANcoder) {
            cancoder = new CANcoder(cancoderId, PhoenixCANBus.canivore());
            cancoderConfig = new CANcoderConfiguration()
                    .withMagnetSensor(new MagnetSensorConfigs()
                            .withMagnetOffset(Radians.of(
                                    IntakeConstants.encoderOffset.get().in(Degree)))
                            .withSensorDirection(IntakeConstants.cancoderSensorDirection));
            PhoenixUtil.tryUntilOk(5, () -> cancoder.getConfigurator().apply(cancoderConfig));

            feedbackConfigs
                    .withSensorToMechanismRatio(1.0)
                    .withRotorToSensorRatio(IntakeConstants.pivotReduction)
                    .withFeedbackSensorSource(feedbackSensorSource)
                    .withFeedbackRemoteSensorID(cancoder.getDeviceID());

        } else {
            feedbackConfigs.withSensorToMechanismRatio(IntakeConstants.pivotReduction);

            cancoder = null;
            cancoderConfig = null;
        }

        motorConfig = new TalonFXConfiguration()
                .withMotorOutput(new MotorOutputConfigs()
                        .withNeutralMode(NeutralModeValue.Brake)
                        .withInverted(IntakeConstants.motorInvertedValue))
                .withFeedback(feedbackConfigs)
                .withClosedLoopGeneral(new ClosedLoopGeneralConfigs().withContinuousWrap(false))
                .withMotionMagic(new MotionMagicConfigs()
                        .withMotionMagicCruiseVelocity(motionMagicMaxVelocity.get())
                        .withMotionMagicAcceleration(motionMagicMaxAcceleration.get())
                        .withMotionMagicJerk(motionMagicMaxJerk.get())
                        .withMotionMagicExpo_kV(pivotKV.get())
                        .withMotionMagicExpo_kA(pivotKA.get()))
                .withSlot0(new Slot0Configs()
                        .withKG(pivotKG.get())
                        .withGravityType(GravityTypeValue.Arm_Cosine)
                        .withKS(pivotKS.get())
                        .withKV(pivotKV.get())
                        .withKA(pivotKA.get())
                        .withKP(motionMagicKP.get())
                        .withKD(motionMagicKD.get()))
                .withSlot1(new Slot1Configs()
                        .withKG(pivotKG.get())
                        .withGravityType(GravityTypeValue.Arm_Cosine)
                        .withKS(pivotKS.get())
                        .withKV(pivotKV.get())
                        .withKA(pivotKA.get())
                        .withKP(pivotKP.get())
                        .withKD(pivotKD.get()))
                .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
                        .withForwardSoftLimitEnable(true)
                        .withForwardSoftLimitThreshold(IntakeConstants.maximumAngle)
                        .withReverseSoftLimitEnable(true)
                        .withReverseSoftLimitThreshold(IntakeConstants.minimumAngle))
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(Amps.of(90))
                        .withSupplyCurrentLimit(Amps.of(40))
                        .withSupplyCurrentLowerLimit(30));

        // Apply configuration to pivot motor
        PhoenixUtil.tryUntilOk(5, () -> pivotMotor.getConfigurator().apply(motorConfig));

        // Configure roller motor separately
        TalonFXConfiguration rollerConfig = new TalonFXConfiguration()
                .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast));
        PhoenixUtil.tryUntilOk(5, () -> rollerMotor.getConfigurator().apply(rollerConfig));

        rollerPositionSignal = rollerMotor.getPosition();
        rollerVelocitySignal = rollerMotor.getVelocity();
        rollerVoltageSignal = rollerMotor.getMotorVoltage();
        rollerStatorCurrentSignal = rollerMotor.getStatorCurrent();
        rollerSupplyCurrentSignal = rollerMotor.getSupplyCurrent();
        rollerTemperatureSignal = rollerMotor.getDeviceTemp();
        pivotPositionSignal = pivotMotor.getPosition();
        pivotVelocitySignal = pivotMotor.getRotorVelocity();
        pivotVoltageSignal = pivotMotor.getMotorVoltage();
        pivotStatorCurrentSignal = pivotMotor.getStatorCurrent();
        pivotSupplyCurrentSignal = pivotMotor.getSupplyCurrent();
        pivotTemperatureSignal = pivotMotor.getDeviceTemp();
        pivotClosedLoopReferenceSignal = pivotMotor.getClosedLoopReference();

        signals.addSignals(
                rollerPositionSignal,
                rollerVelocitySignal,
                rollerVoltageSignal,
                rollerStatorCurrentSignal,
                rollerSupplyCurrentSignal,
                rollerTemperatureSignal,
                pivotPositionSignal,
                pivotVelocitySignal,
                pivotVoltageSignal,
                pivotStatorCurrentSignal,
                pivotSupplyCurrentSignal,
                pivotTemperatureSignal,
                pivotClosedLoopReferenceSignal);

        if (cancoder != null) {
            pivotAngleSignal = cancoder.getAbsolutePosition();
            signals.addSignals(pivotAngleSignal);
        } else {
            pivotAngleSignal = pivotPositionSignal;
        }

        signals.setUpdateFrequencyForAll(Constants.mainLoopFrequency);

        pivotMotor.optimizeBusUtilization();
        rollerMotor.optimizeBusUtilization();

        if (cancoder != null) {
            cancoder.optimizeBusUtilization();
        }
    }

    @Override
    public void updateInputs(IntakeInputs inputs) {
        signals.refreshAll();

        // Update roller inputs
        inputs.rollerPosition = rollerPositionSignal.getValue();
        inputs.rollerVelocity = rollerVelocitySignal.getValue();
        inputs.rollerVoltage = rollerVoltageSignal.getValue();
        inputs.rollerStatorCurrent = rollerStatorCurrentSignal.getValue();
        inputs.rollerSupplyCurrent = rollerSupplyCurrentSignal.getValue();
        inputs.rollerTemperature = rollerTemperatureSignal.getValueAsDouble();

        // Update pivot inputs
        inputs.pivotAngle = pivotAngleSignal.getValue();
        inputs.pivotPosition = pivotPositionSignal.getValue();
        inputs.pivotVelocity = pivotVelocitySignal.getValue();
        inputs.pivotVoltage = pivotVoltageSignal.getValue();
        inputs.pivotStatorCurrent = pivotStatorCurrentSignal.getValue();
        inputs.pivotSupplyCurrent = pivotSupplyCurrentSignal.getValue();
        inputs.pivotTemperature = pivotTemperatureSignal.getValueAsDouble();
        inputs.pivotClosedLoopReference = Units.Rotations.of(pivotClosedLoopReferenceSignal.getValue());

        LoggedTunableNumber.ifChanged(
                0,
                () -> {
                    if (cancoder != null) {
                        cancoderConfig.MagnetSensor.withMagnetOffset(
                                Radians.of(IntakeConstants.encoderOffset.get().in(Degree)));
                        PhoenixUtil.tryUntilOk(
                                5, () -> cancoder.getConfigurator().apply(cancoderConfig));
                    }
                },
                IntakeConstants.encoderOffset);

        LoggedTunableValue.ifChanged(
                0,
                () -> {
                    motorConfig.MotionMagic.MotionMagicCruiseVelocity = motionMagicMaxVelocity.get();
                    motorConfig.MotionMagic.MotionMagicAcceleration = motionMagicMaxAcceleration.get();
                    motorConfig.MotionMagic.MotionMagicJerk = motionMagicMaxJerk.get();
                    motorConfig.MotionMagic.MotionMagicExpo_kV = pivotKV.get();
                    motorConfig.MotionMagic.MotionMagicExpo_kA = pivotKA.get();
                    motorConfig.Slot0.kG = pivotKG.get();
                    motorConfig.Slot0.kS = pivotKS.get();
                    motorConfig.Slot0.kV = pivotKV.get();
                    motorConfig.Slot0.kA = pivotKA.get();
                    motorConfig.Slot0.kP = motionMagicKP.get();
                    motorConfig.Slot0.kD = motionMagicKD.get();
                    motorConfig.Slot1.kG = pivotKG.get();
                    motorConfig.Slot1.kS = pivotKS.get();
                    motorConfig.Slot1.kV = pivotKV.get();
                    motorConfig.Slot1.kA = pivotKA.get();
                    motorConfig.Slot1.kP = pivotKP.get();
                    motorConfig.Slot1.kD = pivotKD.get();
                    PhoenixUtil.tryUntilOk(5, () -> pivotMotor.getConfigurator().apply(motorConfig));
                },
                motionMagicMaxVelocity,
                motionMagicMaxAcceleration,
                motionMagicMaxJerk,
                motionMagicKP,
                motionMagicKD,
                pivotKG,
                pivotKS,
                pivotKV,
                pivotKA,
                pivotKP,
                pivotKD);
    }

    @Override
    public void setRollerOutput(double value) {
        rollerMotor.setControl(rollerDutyCycleControlRequest.withOutput(value));
    }

    @Override
    public void setPivotOutput(double value) {
        pivotMotor.setControl(pivotDutyCycleControlRequest.withOutput(value));
    }

    @Override
    public void setPivotAngle(Angle position) {
        if (useMotionMagic.get()) {
            pivotMotor.setControl(pivotMotionMagicControlRequest.withPosition(position));
        } else {
            pivotMotor.setControl(pivotPositionControlRequest.withPosition(position));
        }
    }

    @Override
    public void setPivotNeutralMode(NeutralModeValue mode) {
        pivotMotor.setNeutralMode(mode);
    }

    @Override
    public void stop() {
        rollerMotor.stopMotor();
        pivotMotor.stopMotor();
    }
}
