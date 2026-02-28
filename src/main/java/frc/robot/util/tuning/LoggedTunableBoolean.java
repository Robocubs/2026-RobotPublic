// Copyright (c) 2026 FRC Team 1701 Robocubs
// http://github.com/Robocubs
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util.tuning;

import java.util.function.BooleanSupplier;

import frc.robot.Constants;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class LoggedTunableBoolean extends LoggedTunableValue<Boolean> implements BooleanSupplier {
    private final String key;
    private boolean hasDefault = false;
    private boolean defaultValue;
    private LoggedNetworkBoolean dashboardBoolean;

    /**
     * Create a new LoggedTunableBoolean
     *
     * @param dashboardKey Key on dashboard
     */
    public LoggedTunableBoolean(String dashboardKey) {
        this.key = tableKey + "/" + dashboardKey;
    }

    /**
     * Create a new LoggedTunableBoolean with the default value
     *
     * @param dashboardKey Key on dashboard
     * @param defaultValue Default value
     */
    public LoggedTunableBoolean(String dashboardKey, boolean defaultValue) {
        this(dashboardKey);
        initDefault(defaultValue);
    }

    /**
     * Set the default value of the boolean. The default value can only be set once.
     *
     * @param defaultValue The default value
     */
    public void initDefault(boolean defaultValue) {
        if (!hasDefault) {
            hasDefault = true;
            this.defaultValue = defaultValue;
            if (Constants.tuningEnabled) {
                dashboardBoolean = new LoggedNetworkBoolean(key, defaultValue);
            }
        }
    }

    /**
     * Get the current value, from dashboard if available and in tuning mode.
     *
     * @return The current value
     */
    public Boolean get() {
        if (!hasDefault) {
            return false;
        } else {
            return Constants.tuningEnabled ? dashboardBoolean.get() : defaultValue;
        }
    }

    /**
     * Use {@link #get()} instead
     */
    @Override
    @Deprecated
    public boolean getAsBoolean() {
        return get();
    }
}
