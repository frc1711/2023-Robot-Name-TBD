// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package util;

import java.util.Optional;

import edu.wpi.first.math.filter.Debouncer;

/** Add your docs here. */
public class NumericDebouncer {

    private final Debouncer debouncer;
    private Optional<Double> lastMeasurement = Optional.empty();

    public NumericDebouncer (Debouncer debouncer) {
        this.debouncer = debouncer;
    }

    
    public Optional<Double> calculate (Optional<Double> measurement) {
        /**
         * Utilizes the WPILib Debouncer class and extends to use doubles
         */
        boolean hasMeasurement = debouncer.calculate(measurement.isPresent());

        if (hasMeasurement) {
            if (measurement.isPresent())
                lastMeasurement = measurement;
        } else lastMeasurement = Optional.empty();

        return lastMeasurement;
    }
}
