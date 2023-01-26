package frc.robot.commands;

public interface InputCurve {
    
    public static final InputCurve THREE_HALVES_CURVE = (double input) -> Math.pow(input, 1.5);
    public static final InputCurve SQUARE_CURVE = (double input) -> input * input;
    public static final InputCurve NO_CURVE = (double input) -> input;
    
    /**
     * Applies a given input curve to {@code input}. Negative inputs are negated when fed into
     * the input curve so that the output is symmetrical across x=0. {@code input} values beyond the range
     * [-1, 1] are capped at -1 or 1.
     * @param curve The {@link InputCurve} to apply to the input.
     * @param input The {@code double} value to which the input curve is applied.
     * @return The input adjusted according to the curve.
     */
    public static double apply (InputCurve curve, double input) {
        // Limit to [-1, 1]
        if (input < -1) input = -1;
        if (input >  1) input =  1;
        
        // Negate output for negative values
        if (input < 0)
            return -curve.getValue(-input);
        else
            return curve.getValue(input);
    }
    
    public static Input2D apply (InputCurve curve, Input2D input) {
        // Get the magnitude of the input and apply the curve to get a new (adjusted) magnitude
        double magnitude = input.getMagnitude();
        double adjustedMagnitude = apply(curve, magnitude);
        
        // Do not divide by zero, but return the input scaled so that it has the new adjusted magnitude
        if (magnitude == 0) return new Input2D(0, 0);
        return input.scale(adjustedMagnitude / magnitude);
    }
    
    public static Input2D apply (InputCurve curve, double x, double y) {
        return apply(curve, new Input2D(x, y));
    }
    
    public default InputCurve withDeadband (double deadband) {
        return (double input) -> {
            
            // Apply a linear transformation to the input to adjust it for the deadband
            input -= deadband;
            input /= (1 - deadband);
            
            // Do not allow the input to be less than zero
            if (input < 0) input = 0;
            
            // Apply this input curve after the deadband
            return this.getValue(input);
            
        };
    }
    
    /**
     * Get the output of the curve corresponding to a given input.
     * This method should only be called directly if {@code input} is guaranteed to be between 0 and 1.
     * Otherwise, use {@link InputCurve#apply(InputCurve, double)}.<br></br>
     * As a general rule, an input of 0 should map to 0, and an input of 1 should map to 1.
     * @param input A {@code double} value on the interval [0, 1].
     * @return The {@code input} after adjusting for the curve.
     */
    public double getValue (double input);
    
    public static record Input2D (double x, double y) {
    
        public double getMagnitude () {
            return Math.sqrt(x*x + y*y);
        }
        
        public Input2D scale (double k) {
            return new Input2D(x*k, y*k);
        }
        
    }
    
}
