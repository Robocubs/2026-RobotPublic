// Copyright (c) 2026 FRC Team 1701 Robocubs
// http://github.com/Robocubs
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.struct.StructSerializable;
import frc.robot.RobotState.VisionMeasurement;
import frc.robot.simulation.ReplayRobotState;
import frc.robot.subsystems.drive.struct.SwerveDriveInputsStruct;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import static edu.wpi.first.units.Units.*;

public interface DriveIO {

    public class DriveIOInputs implements LoggableInputs {
        public SwerveDriveInputs[] swerve = new SwerveDriveInputs[0];
        public Rotation3d rotation = Rotation3d.kZero;
        public boolean gyroConnected;

        @Override
        public void toLog(LogTable table) {
            table.put("Swerve", swerve);
            table.put("Rotation", rotation);
            table.put("GyroConnected", gyroConnected);
        }

        @Override
        public void fromLog(LogTable table) {
            swerve = table.get("Swerve", swerve);
            rotation = table.get("Rotation", rotation);
            gyroConnected = table.get("GyroConnected", gyroConnected);

            swerve = ReplayRobotState.getInstance().update(swerve);
        }
    }

    @AutoLog
    public class ModuleIOInputs implements StructSerializable {
        public AngularVelocity driveVelocity = RadiansPerSecond.zero();
        public Angle drivePosition = Radians.zero();
        public Current driveSupplyCurrent = Amps.zero();
        public Current driveStatorCurrent = Amps.zero();
        public Voltage driveVoltage = Volts.zero();
        public double driveTemperature;

        public AngularVelocity steerVelocity = RadiansPerSecond.zero();
        public Angle steerPosition = Radians.zero();
        public Current steerSupplyCurrent = Amps.zero();
        public Current steerStatorCurrent = Amps.zero();
        public Voltage steerVoltage = Volts.zero();
        public double steerTemperature;
    }

    @RequiredArgsConstructor
    public class SwerveDriveInputs implements StructSerializable {
        public final Pose2d pose;
        public final ChassisSpeeds speeds;
        public final SwerveModuleState[] moduleStates;
        public final SwerveModuleState[] moduleTargets;
        public final SwerveModulePosition[] modulePositions;
        public final Rotation2d rawHeading;
        public final double timestamp;
        public final double odometryPeriod;
        public final int successfulDaqs;
        public final int failedDaqs;

        public static SwerveDriveInputs fromSwerveDriveState(SwerveDriveState state) {
            return new SwerveDriveInputs(
                    state.Pose,
                    state.Speeds,
                    state.ModuleStates,
                    state.ModuleTargets,
                    state.ModulePositions,
                    state.RawHeading,
                    Utils.currentTimeToFPGATime(state.Timestamp),
                    state.OdometryPeriod,
                    state.SuccessfulDaqs,
                    state.FailedDaqs);
        }

        /** SwerveDriveInputs struct for serialization. */
        public static final SwerveDriveInputsStruct struct = new SwerveDriveInputsStruct();
    }

    public default void updateInputs(
            DriveIOInputs driveInputs,
            ModuleIOInputs frontLeftModuleInputs,
            ModuleIOInputs frontRightModuleInputs,
            ModuleIOInputs backLeftModuleInputs,
            ModuleIOInputs backRightModuleInputs) {}

    public default void setControl(SwerveRequest request) {}

    public default void resetPose(Pose2d pose2d) {}

    public default void resetRotation(Rotation2d rotation2d) {}

    public default void setNeutralMode(NeutralModeValue neutralMode) {}

    public default void addVisionMeasurements(VisionMeasurement[] measurements) {}

    public default void setBrownoutProtection(boolean enabled) {}
}
