// Copyright (c) 2026 FRC Team 1701 Robocubs
// http://github.com/Robocubs
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util;

import java.util.function.Supplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.Timestamp;
import com.ctre.phoenix6.Utils;
import edu.wpi.first.wpilibj.Timer;

public class PhoenixUtil {
    private static final int defaultMaxAttempts = 5;

    public static void tryUntilOk(Supplier<StatusCode> command) {
        tryUntilOk(defaultMaxAttempts, command);
    }

    public static void tryUntilOk(int maxAttempts, Supplier<StatusCode> command) {
        for (int i = 0; i < maxAttempts; i++) {
            if (command.get().isOK()) break;
        }
    }

    public static double getFPGATimestamp(BaseStatusSignal signal) {
        return toFPGATimestamp(signal.getTimestamp().getTime());
    }

    public static double toFPGATimestamp(Timestamp timestamp) {
        return toFPGATimestamp(timestamp.getTime());
    }

    public static double toFPGATimestamp(double timestamp) {
        return timestamp + Timer.getFPGATimestamp() - Utils.getCurrentTimeSeconds();
    }
}
