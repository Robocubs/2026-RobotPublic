// Copyright (c) 2026 FRC Team 1701 Robocubs
// http://github.com/Robocubs
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.RobotBase;

import static edu.wpi.first.units.Units.*;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
    private static final Mode simMode =
            RobotBase.isReal() || System.getenv("AKIT_LOG_PATH") == null ? Mode.SIM : Mode.REPLAY;
    private static final RobotType realBot = RobotType.COMP_BOT;
    private static final RobotType simBot = RobotType.SIM_BOT;
    private static final boolean realBotTuningEnabled = true;
    private static final boolean realBotCharacterizationEnabled = true;
    private static final boolean simBotCharacterizationEnabled = true;
    private static final boolean useMapleSim = true;

    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;
    public static final RobotType robot = (currentMode == Mode.REAL || currentMode == Mode.REPLAY) ? realBot : simBot;
    public static final boolean tuningEnabled = robot == RobotType.SIM_BOT ? true : realBotTuningEnabled;
    public static final boolean characterizationEnabled =
            robot == RobotType.SIM_BOT ? simBotCharacterizationEnabled : realBotCharacterizationEnabled;
    public static final boolean mapleSimEnabled = currentMode == Mode.SIM && useMapleSim;

    public static enum RobotType {
        COMP_BOT,
        SIM_BOT
    }

    public static enum Mode {
        REAL,
        REPLAY,
        SIM
    }

    public static final Time mainLoopPeriod = Milliseconds.of(20);
    public static final Frequency mainLoopFrequency = mainLoopPeriod.asFrequency();
    public static final double epsilon = 1e-12;
}
