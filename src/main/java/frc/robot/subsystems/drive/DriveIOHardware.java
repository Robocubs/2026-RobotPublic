// Copyright (c) 2026 FRC Team 1701 Robocubs
// http://github.com/Robocubs
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.drive;

import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Function;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.StatusSignalCollection;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.RobotState.VisionMeasurement;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;

import static frc.robot.subsystems.drive.DriveConstants.*;

public class DriveIOHardware implements DriveIO {
    protected final SwerveDrivetrain<TalonFX, TalonFX, CANcoder> swerveDrivetrain;
    private final Queue<SwerveDriveInputs> inputsQueue = new ArrayBlockingQueue<>(20);
    private final Lock inputsLock = new ReentrantLock();
    private final StatusSignalCollection statusSignals = new StatusSignalCollection();
    private final ModuleSignals frontLeftModuleSignals;
    private final ModuleSignals frontRightModuleSignals;
    private final ModuleSignals backLeftModuleSignals;
    private final ModuleSignals backRightModuleSignals;

    private record ModuleSignals(
            StatusSignal<AngularVelocity> driveVelocity,
            StatusSignal<Angle> drivePosition,
            StatusSignal<Current> driveSupplyCurrent,
            StatusSignal<Current> driveStatorCurrent,
            StatusSignal<Voltage> driveVoltage,
            StatusSignal<Temperature> driveTemperature,
            StatusSignal<AngularVelocity> steerVelocity,
            StatusSignal<Angle> steerPosition,
            StatusSignal<Current> steerSupplyCurrent,
            StatusSignal<Current> steerStatorCurrent,
            StatusSignal<Voltage> steerVoltage,
            StatusSignal<Temperature> steerTemperature) {}

    public DriveIOHardware() {
        regulateModuleConstants(moduleConstants);

        swerveDrivetrain =
                new TunerSwerveDrivetrain(drivetrainConstants, 250, odometryStdDevs, visionStdDevs, moduleConstants);
        swerveDrivetrain.registerTelemetry(this::addTelemetry);

        Function<Integer, ModuleSignals> createModuleSignals = (index) -> {
            var module = swerveDrivetrain.getModule(index);
            var driveMotor = module.getDriveMotor();
            var steerMotor = module.getSteerMotor();
            var signals = new ModuleSignals(
                    driveMotor.getVelocity(),
                    driveMotor.getPosition(),
                    driveMotor.getSupplyCurrent(),
                    driveMotor.getStatorCurrent(),
                    driveMotor.getMotorVoltage(),
                    driveMotor.getDeviceTemp(),
                    steerMotor.getVelocity(),
                    steerMotor.getPosition(),
                    steerMotor.getSupplyCurrent(),
                    steerMotor.getStatorCurrent(),
                    steerMotor.getMotorVoltage(),
                    steerMotor.getDeviceTemp());
            statusSignals.addSignals(
                    signals.driveVelocity,
                    signals.drivePosition,
                    signals.driveSupplyCurrent,
                    signals.driveStatorCurrent,
                    signals.driveVoltage,
                    signals.driveTemperature,
                    signals.steerVelocity,
                    signals.steerPosition,
                    signals.steerSupplyCurrent,
                    signals.steerStatorCurrent,
                    signals.steerVoltage,
                    signals.steerTemperature);
            return signals;
        };

        frontLeftModuleSignals = createModuleSignals.apply(0);
        frontRightModuleSignals = createModuleSignals.apply(1);
        backLeftModuleSignals = createModuleSignals.apply(2);
        backRightModuleSignals = createModuleSignals.apply(3);
    }

    protected void regulateModuleConstants(SwerveModuleConstants<?, ?, ?>[] constants) {}

    @Override
    public void updateInputs(
            DriveIOInputs driveInputs,
            ModuleIOInputs frontLeftModuleInputs,
            ModuleIOInputs frontRightModuleInputs,
            ModuleIOInputs backLeftModuleInputs,
            ModuleIOInputs backRightModuleInputs) {
        inputsLock.lock();
        try {
            driveInputs.swerve = inputsQueue.toArray(new DriveIO.SwerveDriveInputs[inputsQueue.size()]);
            inputsQueue.clear();
        } finally {
            inputsLock.unlock();
        }

        driveInputs.rotation = swerveDrivetrain.getRotation3d();

        statusSignals.refreshAll();
        updateModuleInputs(frontLeftModuleSignals, frontLeftModuleInputs);
        updateModuleInputs(frontRightModuleSignals, frontRightModuleInputs);
        updateModuleInputs(backLeftModuleSignals, backLeftModuleInputs);
        updateModuleInputs(backRightModuleSignals, backRightModuleInputs);
    }

    private void updateModuleInputs(ModuleSignals signals, ModuleIOInputs inputs) {
        inputs.driveVelocity = signals.driveVelocity.getValue();
        inputs.drivePosition = signals.drivePosition.getValue();
        inputs.driveSupplyCurrent = signals.driveSupplyCurrent.getValue();
        inputs.driveStatorCurrent = signals.driveStatorCurrent.getValue();
        inputs.driveVoltage = signals.driveVoltage.getValue();
        inputs.driveTemperature = signals.driveTemperature.getValueAsDouble();

        inputs.steerVelocity = signals.steerVelocity.getValue();
        inputs.steerPosition = signals.steerPosition.getValue();
        inputs.steerSupplyCurrent = signals.steerSupplyCurrent.getValue();
        inputs.steerStatorCurrent = signals.steerStatorCurrent.getValue();
        inputs.steerVoltage = signals.steerVoltage.getValue();
        inputs.steerTemperature = signals.steerTemperature.getValueAsDouble();
    }

    private void addTelemetry(SwerveDriveState state) {
        var inputs = SwerveDriveInputs.fromSwerveDriveState(state);
        inputsLock.lock();
        try {
            inputsQueue.add(inputs);
        } finally {
            inputsLock.unlock();
        }
    }

    @Override
    public void setControl(SwerveRequest request) {
        swerveDrivetrain.setControl(request);
        swerveDrivetrain.getOperatorForwardDirection();
    }

    @Override
    public void resetPose(Pose2d pose2d) {
        swerveDrivetrain.resetPose(pose2d);
    }

    @Override
    public void resetRotation(Rotation2d rotation2d) {
        swerveDrivetrain.resetRotation(rotation2d);
    }

    @Override
    public void setNeutralMode(NeutralModeValue neutralMode) {
        swerveDrivetrain.configNeutralMode(neutralMode);
    }

    @Override
    public void addVisionMeasurements(VisionMeasurement[] measurements) {
        for (var measurement : measurements) {
            swerveDrivetrain.addVisionMeasurement(
                    measurement.pose.toPose2d(),
                    Utils.fpgaToCurrentTime(measurement.timestampSeconds),
                    measurement.stdDevs);
        }
    }
}
