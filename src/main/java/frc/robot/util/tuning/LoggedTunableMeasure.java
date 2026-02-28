// Copyright (c) 2026 FRC Team 1701 Robocubs
// http://github.com/Robocubs
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util.tuning;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Unit;
import frc.robot.Constants;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * Class for a tunable number. Gets value from dashboard in tuning mode, returns default if not or
 * value not in dashboard.
 */
public class LoggedTunableMeasure<U extends Unit, M extends Measure<U>> extends LoggedTunableValue<M> {
    private final String key;
    private boolean hasDefault = false;
    private M defaultValue;
    private LoggedNetworkNumber dashboardNumber;
    private U unit;
    private MutableMeasure<?, ?, ?> value;

    /**
     * Create a new LoggedTunableNumber
     *
     * @param dashboardKey Key on dashboard
     */
    public LoggedTunableMeasure(String dashboardKey, U unit) {
        this.key = tableKey + "/" + dashboardKey;
        this.unit = unit;
        this.value = unit.mutable(0);
    }

    /**
     * Create a new LoggedTunableNumber with the default value
     *
     * @param dashboardKey Key on dashboard
     * @param defaultValue Default value
     */
    public LoggedTunableMeasure(String dashboardKey, M defaultValue) {
        this(dashboardKey, defaultValue.unit());
        initDefault(defaultValue);
    }

    /**
     * Set the default value of the number. The default value can only be set once.
     *
     * @param defaultValue The default value
     */
    public void initDefault(M defaultValue) {
        if (!hasDefault) {
            hasDefault = true;
            this.defaultValue = defaultValue;
            if (Constants.tuningEnabled) {
                dashboardNumber = new LoggedNetworkNumber(key, defaultValue.in(unit));
            }
        }
    }

    /**
     * Get the current value, from dashboard if available and in tuning mode.
     *
     * @return The current value
     */
    @SuppressWarnings("unchecked")
    public M get() {
        if (!hasDefault) {
            return (M) unit.zero();
        } else if (Constants.tuningEnabled) {
            value.mut_setMagnitude(dashboardNumber.get());
            return (M) value;
        } else {
            return defaultValue;
        }
    }
}
