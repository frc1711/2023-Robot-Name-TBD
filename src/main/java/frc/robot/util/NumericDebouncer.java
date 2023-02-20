package frc.robot.util;

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
