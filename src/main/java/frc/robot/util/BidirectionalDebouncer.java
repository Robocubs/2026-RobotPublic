// Copyright (c) 2026 FRC Team 1701 Robocubs
// http://github.com/Robocubs
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.wpilibj.Timer;

public class BidirectionalDebouncer {
    private double debounceRisingTimeSeconds;
    private double debounceFallingTimeSeconds;
    private boolean baseline;

    private double previousTimeSeconds;

    public BidirectionalDebouncer(double debounceRisingTimeSeconds, double debounceFallingTimeSeconds) {
        this(debounceRisingTimeSeconds, debounceFallingTimeSeconds, false);
    }

    public BidirectionalDebouncer(
            double debounceRisingTimeSeconds, double debounceFallingTimeSeconds, boolean baseline) {
        this.debounceRisingTimeSeconds = debounceRisingTimeSeconds;
        this.debounceFallingTimeSeconds = debounceFallingTimeSeconds;
        this.baseline = baseline;
    }

    public void reset(boolean baseline) {
        this.baseline = baseline;
        resetTimer();
    }

    private void resetTimer() {
        previousTimeSeconds = MathSharedStore.getTimestamp();
    }

    private boolean hasElapsedRising() {
        return Timer.getTimestamp() - previousTimeSeconds >= debounceRisingTimeSeconds;
    }

    private boolean hasElapsedFalling() {
        return Timer.getTimestamp() - previousTimeSeconds >= debounceFallingTimeSeconds;
    }

    public boolean calculate(boolean input) {
        if (input && hasElapsedRising()) {
            baseline = input;
            resetTimer();
        } else if (!input && hasElapsedFalling()) {
            baseline = input;
            resetTimer();
        }

        return baseline;
    }

    public boolean getBaseline() {
        return baseline;
    }
}
