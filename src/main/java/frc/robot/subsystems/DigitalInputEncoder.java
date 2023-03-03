package frc.robot.subsystems;

import claw.math.Transform;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycle;

public class DigitalInputEncoder implements AutoCloseable {
    
    private final DutyCycle input;
    
    /**
     * Invert BEFORE applying dutyCycleToDegrees if it has a negative relationship
     */
    private final Transform dutyCycleToDegrees;
    
    private final boolean hasPositiveRelationship;
    
    /**
     * 
     * @param source
     * @param hasPositiveRelationship    Whether or not, when the value of the duty cycle increases, the output angle should increase as a result
     */
    public DigitalInputEncoder (DutyCycle input, boolean hasPositiveRelationship, AnglePoint p1, AnglePoint p2) {
        this.input = input;
        this.hasPositiveRelationship = hasPositiveRelationship;
        
        // Inverting the duty cycle so that the transform is fed a positive relationship
        if (!hasPositiveRelationship) {
            p1 = new AnglePoint(p1.degrees(), 1 - p1.inputValue());
            p2 = new AnglePoint(p2.degrees(), 1 - p2.inputValue());
        }
        
        // Change the points so that the second one has higher degrees than the first
        dutyCycleToDegrees =
            (p1.degrees() < p2.degrees())
                ? getTransformFor(p1, p2)
                : getTransformFor(p2, p1);
    }
    
    public record AnglePoint (double degrees, double inputValue) { }
    
    /**
     * lowPoint has both lower degrees and inputValue than highPoint. If the highPoint has a lower input
     * value, it's just because it wrapped around from 1 back to 0
     */
    private Transform getTransformFor (AnglePoint lowPoint, AnglePoint highPoint) {
        
        // Part 1
        // The goal of this part is to make the relationship from duty cycle input to degrees output linear
        
        // This transform will change the provided duty cycle input into one which accounts for wrapping from 1 to 0
        // (If necessary)
        Transform adjustDutyCycleInput = i -> i;
        if (highPoint.inputValue() < lowPoint.inputValue()) {
            
            // Must account for wrapping of 1 back to 0:
            // 0 . . . highPoint       middleInput     lowPoint . . . 1
            //                    ^^ considered high
            //                                      ^^ considered low
            
            // Get the middle point between the input values
            double middleInput = (lowPoint.inputValue() + highPoint.inputValue()) / 2;
            
            // Give the highPoint a input value of 1 higher to compensate for wrapping when we create the interpolator
            highPoint = new AnglePoint(highPoint.degrees(), highPoint.inputValue() + 1);
            
            // Wrap points below the middleInput to be above 1 to compensate
            adjustDutyCycleInput = i -> (i < middleInput) ? (i + 1) : (i);
        }
        
        // Part 2
        // Create the interpolator for the linear relationship
        
        Interpolator adjustedDutyCycleToDegrees = new Interpolator(
            lowPoint.inputValue(), lowPoint.degrees(),
            highPoint.inputValue(), highPoint.degrees()
        );
        
        return adjustDutyCycleInput.then(adjustedDutyCycleToDegrees::get);
        
    }
    
    /**
     * Raw duty cycle from 0 to 1
     */
    public double getRawDutyCycleValue () {
        return input.getOutput();
    }
    
    public double getRotationDegrees () {
        double input = getRawDutyCycleValue();
        
        // Invert input if there's a negative relationship
        if (!hasPositiveRelationship) input = 1 - input;
        
        // Return degrees
        return dutyCycleToDegrees.apply(input);
    }
    
    public Rotation2d getRotation () {
        return Rotation2d.fromDegrees(getRotationDegrees());
    }
    
    @Override
    public void close () {
        input.close();
    }
    
    private static class Interpolator {
        
        private final double x1, xRange, y1, yRange;
        
        public Interpolator (double x1, double y1, double x2, double y2) {
            this.x1 = x1;
            xRange = x2 - x1;
            this.y1 = y1;
            yRange = y2 - y1;
        }
        
        public double get (double x) {
            double p = (x - x1) / xRange;
            return y1 + p * yRange;
        }
        
    }
    
}
