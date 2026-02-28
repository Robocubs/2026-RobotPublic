// Copyright (c) 2026 FRC Team 1701 Robocubs
// http://github.com/Robocubs
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.vision.apriltag;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.util.protobuf.ProtobufSerializable;
import frc.robot.subsystems.vision.apriltag.proto.AprilTagTargetProto;

public class AprilTagTarget implements ProtobufSerializable {
    public final int id;
    public final Transform3d bestCameraToTarget;
    public final Transform3d altCameraToTarget;
    public final double ambiguity;

    public AprilTagTarget(int id, Transform3d bestCameraToTarget, Transform3d altCameraToTarget, double ambiguity) {
        this.id = id;
        this.bestCameraToTarget = bestCameraToTarget;
        this.altCameraToTarget = altCameraToTarget;
        this.ambiguity = ambiguity;
    }

    public static final AprilTagTargetProto proto = new AprilTagTargetProto();
}
