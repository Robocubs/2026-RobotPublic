// Copyright (c) 2026 FRC Team 1701 Robocubs
// http://github.com/Robocubs
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.drive.struct;

import java.nio.ByteBuffer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.struct.Struct;
import frc.robot.subsystems.drive.DriveIO.SwerveDriveInputs;

public class SwerveDriveInputsStruct implements Struct<SwerveDriveInputs> {
    @Override
    public Class<SwerveDriveInputs> getTypeClass() {
        return SwerveDriveInputs.class;
    }

    @Override
    public String getTypeName() {
        return "SwerveDriveInputs";
    }

    @Override
    public int getSize() {
        return Pose2d.struct.getSize()
                + ChassisSpeeds.struct.getSize()
                + (SwerveModuleState.struct.getSize() * 4)
                + (SwerveModuleState.struct.getSize() * 4)
                + (SwerveModulePosition.struct.getSize() * 4)
                + Rotation2d.struct.getSize()
                + kSizeDouble
                + kSizeDouble
                + kSizeInt32
                + kSizeInt32;
    }

    @Override
    public String getSchema() {
        return "Pose2d pose;ChassisSpeeds speeds;SwerveModuleState moduleStates[4];SwerveModuleState targetStates[4];SwerveModulePosition modulePositions[4];Rotation2d rawHeading;double timestamp;double odometryPeriod;int32 successfulDaqs;int32 failedDaqs";
    }

    @Override
    public Struct<?>[] getNested() {
        return new Struct<?>[] {SwerveModuleState.struct, SwerveModulePosition.struct, Rotation2d.struct};
    }

    @Override
    public SwerveDriveInputs unpack(ByteBuffer bb) {
        var pose = Pose2d.struct.unpack(bb);
        var speeds = ChassisSpeeds.struct.unpack(bb);

        var moduleStates = new SwerveModuleState[4];
        for (var i = 0; i < 4; i++) {
            moduleStates[i] = SwerveModuleState.struct.unpack(bb);
        }
        var moduleTargets = new SwerveModuleState[4];
        for (var i = 0; i < 4; i++) {
            moduleTargets[i] = SwerveModuleState.struct.unpack(bb);
        }

        var modulePositions = new SwerveModulePosition[4];
        for (var i = 0; i < 4; i++) {
            modulePositions[i] = SwerveModulePosition.struct.unpack(bb);
        }

        var rawHeading = Rotation2d.struct.unpack(bb);
        var timestamp = bb.getDouble();
        var odometryPeriod = bb.getDouble();
        var successfulDaqs = bb.getInt();
        var failedDaqs = bb.getInt();
        return new SwerveDriveInputs(
                pose,
                speeds,
                moduleStates,
                moduleTargets,
                modulePositions,
                rawHeading,
                timestamp,
                odometryPeriod,
                successfulDaqs,
                failedDaqs);
    }

    @Override
    public void pack(ByteBuffer bb, SwerveDriveInputs value) {
        Pose2d.struct.pack(bb, value.pose);
        ChassisSpeeds.struct.pack(bb, value.speeds);

        for (var moduleState : value.moduleStates) {
            SwerveModuleState.struct.pack(bb, moduleState);
        }
        for (var moduleTarget : value.moduleTargets) {
            SwerveModuleState.struct.pack(bb, moduleTarget);
        }

        for (var modulePosition : value.modulePositions) {
            SwerveModulePosition.struct.pack(bb, modulePosition);
        }

        Rotation2d.struct.pack(bb, value.rawHeading);
        bb.putDouble(value.timestamp);
        bb.putDouble(value.odometryPeriod);
        bb.putInt(value.successfulDaqs);
        bb.putInt(value.failedDaqs);
    }
}
