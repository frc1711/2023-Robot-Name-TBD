package frc.robot.commands.auton;

import claw.math.Transform;
import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveTurnCorrector {
    
    private final Transform yawOffsetToCorrectionTurn =
        // Wrap degrees from -180 to +180
        ((Transform)(deg -> {
            while (deg > 180) deg -= 360;
            while (deg < -180) deg += 360;
            return deg;
        }))
        
        .then(Transform.NEGATE)
        
        // Apply the corrective turn
        .then(offsetDeg -> offsetDeg / 30.)
        .then(Transform.clamp(-1, 1))
        .then(v -> 3.5*v);
    
    public double getCorrectionSpeed (Rotation2d currentRotation, Rotation2d desiredRotation) {
        return yawOffsetToCorrectionTurn.apply(currentRotation.minus(desiredRotation).getDegrees());
    }
    
}
