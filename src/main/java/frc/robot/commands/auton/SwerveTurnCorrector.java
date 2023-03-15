package frc.robot.commands.auton;

import claw.math.input.InputTransform;
import claw.math.Transform;
import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveTurnCorrector {
    
    private static final double OFFSET_MIN = 1;
    
    private final Transform yawOffsetToCorrectionTurn =
        // Wrap degrees from -180 to +180
        ((Transform)(deg -> {
            while (deg > 180) deg -= 360;
            while (deg < -180) deg += 360;
            return deg;
        }))
        
        .then(offsetDeg -> (Math.abs(offsetDeg) > OFFSET_MIN ? offsetDeg : 0))
        
        .then(Transform.NEGATE)
        
        // Apply the corrective turn
        .then(offsetDeg -> offsetDeg / 30.)
        .then(Transform.clamp(-1, 1))
        .then(v -> 3.5*v);
    
    public double getCorrectionSpeed (Rotation2d currentRotation, Rotation2d desiredRotation) {
        return yawOffsetToCorrectionTurn.apply(currentRotation.minus(desiredRotation).getDegrees());
    }
    
}
