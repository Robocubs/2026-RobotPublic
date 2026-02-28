// Copyright (c) 2026 FRC Team 1701 Robocubs
// http://github.com/Robocubs
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util.tuning;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;

public abstract class LoggedTunableValue<T> {
    protected static final String tableKey = "/Tuning";

    private final Map<Integer, T> lastHasChangedValues = new HashMap<>();

    public abstract T get();

    /**
     * Checks whether the value has changed since our last check
     *
     * @param id Unique identifier for the caller to avoid conflicts when shared between multiple
     *     objects. Recommended approach is to pass the result of "hashCode()"
     * @return True if the value has changed since the last time this method was called, false
     *     otherwise.
     */
    public boolean hasChanged(int id) {
        var currentValue = get();
        var lastValue = lastHasChangedValues.get(id);
        if (lastValue == null || !lastValue.equals(currentValue)) {
            lastHasChangedValues.put(id, currentValue);
            return true;
        }

        return false;
    }

    /**
     * Runs action if any of the tunableValues have changed
     *
     * @param id Unique identifier for the caller to avoid conflicts when shared between multiple
     *     objects. Recommended approach is to pass the result of "hashCode()"
     * @param action Callback to run when any of the tunable values have changed
     */
    @SuppressWarnings("rawtypes")
    public static void ifChanged(int id, Runnable action, LoggedTunableValue... tunableValues) {
        if (Arrays.stream(tunableValues).anyMatch(tunableValue -> tunableValue.hasChanged(id))) {
            action.run();
        }
    }
}
