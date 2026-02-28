// Copyright (c) 2026 FRC Team 1701 Robocubs
// http://github.com/Robocubs
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.vision.apriltag.proto;

import com.robocubs.proto.Apriltag.ProtobufAprilTagTarget;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.util.protobuf.Protobuf;
import frc.robot.subsystems.vision.apriltag.AprilTagTarget;
import us.hebi.quickbuf.Descriptors.Descriptor;
import us.hebi.quickbuf.RepeatedMessage;

public class AprilTagTargetProto implements Protobuf<AprilTagTarget, ProtobufAprilTagTarget> {
    @Override
    public Class<AprilTagTarget> getTypeClass() {
        return AprilTagTarget.class;
    }

    @Override
    public Descriptor getDescriptor() {
        return ProtobufAprilTagTarget.getDescriptor();
    }

    @Override
    public ProtobufAprilTagTarget createMessage() {
        return ProtobufAprilTagTarget.newInstance();
    }

    @Override
    public AprilTagTarget unpack(ProtobufAprilTagTarget msg) {
        return new AprilTagTarget(
                msg.getId(),
                Transform3d.proto.unpack(msg.getBestCameraToTarget()),
                Transform3d.proto.unpack(msg.getAltCameraToTarget()),
                msg.getAmbiguity());
    }

    public AprilTagTarget[] unpack(RepeatedMessage<ProtobufAprilTagTarget> msg) {
        var targets = new AprilTagTarget[msg.length()];
        for (int i = 0; i < targets.length; i++) {
            targets[i] = unpack(msg.get(i));
        }
        return targets;
    }

    @Override
    public void pack(ProtobufAprilTagTarget msg, AprilTagTarget value) {
        msg.setId(value.id);
        Transform3d.proto.pack(msg.getMutableBestCameraToTarget(), value.bestCameraToTarget);
        Transform3d.proto.pack(msg.getMutableAltCameraToTarget(), value.altCameraToTarget);
        msg.setAmbiguity(value.ambiguity);
    }

    public void pack(RepeatedMessage<ProtobufAprilTagTarget> msg, AprilTagTarget[] values) {
        var targets = msg.reserve(values.length);
        for (var target : values) {
            pack(targets.next(), target);
        }
    }

    @Override
    public boolean isImmutable() {
        return true;
    }
}
