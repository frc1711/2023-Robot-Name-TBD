// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package util;

import java.util.Optional;

import edu.wpi.first.math.filter.Debouncer;

/** Add your docs here. */
public class NumericDebouncer {

    private final Debouncer debouncer;
    private boolean hasMeasurement = false;
    private double lastMeasurement;
    public NumericDebouncer (Debouncer debouncer) {
        this.debouncer = debouncer;
    }

    
    public Optional<Double> calculate (Optional<Double> measurement) {
        /**
         * Utilizes the WPILib Debouncer class and extends to use doubles
         */
        hasMeasurement = debouncer.calculate(measurement.isPresent());
        if (measurement.isPresent()) lastMeasurement = measurement.get();
        return hasMeasurement ? Optional.of(lastMeasurement) : Optional.empty();
    }
}
