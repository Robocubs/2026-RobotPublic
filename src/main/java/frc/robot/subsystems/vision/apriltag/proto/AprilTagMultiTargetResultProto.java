// Copyright (c) 2026 FRC Team 1701 Robocubs
// http://github.com/Robocubs
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.vision.apriltag.proto;

import java.util.Arrays;

import com.robocubs.proto.Apriltag.ProtobufAprilTagMultiTargetResult;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.util.protobuf.Protobuf;
import frc.robot.subsystems.vision.apriltag.AprilTagMultiTargetResult;
import us.hebi.quickbuf.Descriptors.Descriptor;

public class AprilTagMultiTargetResultProto
        implements Protobuf<AprilTagMultiTargetResult, ProtobufAprilTagMultiTargetResult> {

    @Override
    public Class<AprilTagMultiTargetResult> getTypeClass() {
        return AprilTagMultiTargetResult.class;
    }

    @Override
    public Descriptor getDescriptor() {
        return ProtobufAprilTagMultiTargetResult.getDescriptor();
    }

    @Override
    public ProtobufAprilTagMultiTargetResult createMessage() {
        return ProtobufAprilTagMultiTargetResult.newInstance();
    }

    @Override
    public AprilTagMultiTargetResult unpack(ProtobufAprilTagMultiTargetResult msg) {
        var msgTargetIds = msg.getTargetIds();
        var targetIds = new int[msgTargetIds.length()];
        Arrays.setAll(targetIds, i -> msgTargetIds.get(i));

        var pose = Pose3d.proto.unpack(msg.getCameraPose());
        var reprojectionError = msg.getReprojectionError();

        return new AprilTagMultiTargetResult(targetIds, pose, reprojectionError);
    }

    @Override
    public void pack(ProtobufAprilTagMultiTargetResult msg, AprilTagMultiTargetResult value) {
        var targetIds = msg.getMutableTargetIds();
        for (var id : value.targetIds) {
            targetIds.add(id);
        }

        Pose3d.proto.pack(msg.getMutableCameraPose(), value.cameraPose);
        msg.setReprojectionError(value.reprojectionError);
    }

    @Override
    public boolean isImmutable() {
        return true;
    }
}
