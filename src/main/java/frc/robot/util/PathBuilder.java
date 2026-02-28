// Copyright (c) 2026 FRC Team 1701 Robocubs
// http://github.com/Robocubs
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;

public class PathBuilder {
    private final List<Pose2d> mPath = new ArrayList<>();

    public Pose2d[] build() {
        return mPath.toArray(new Pose2d[mPath.size()]);
    }

    public Pose2d[] buildAndClear() {
        var path = build();
        clear();
        return path;
    }

    public void clear() {
        mPath.clear();
    }

    public PathBuilder add(Pose2d pose) {
        mPath.add(pose);
        return this;
    }

    public PathBuilder add(Pose2d[] path) {
        Collections.addAll(mPath, path);
        return this;
    }
}
