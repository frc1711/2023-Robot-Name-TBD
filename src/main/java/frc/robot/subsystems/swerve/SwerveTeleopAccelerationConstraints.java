package frc.robot.subsystems.swerve;

import claw.math.Transform;
import claw.math.Vector;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N2;

public class SwerveTeleopAccelerationConstraints {
    
    private final double maxStrafeAcceleration;
    private final SlewRateLimiter turningAccelerationLimiter;
    
    private double lastApplicationTime = 0;
    private Vector<N2> lastStrafeVelocity = new Vector<>(Nat.N2(), 0, 0);
    
    public SwerveTeleopAccelerationConstraints (double maxStrafeAcceleration, double maxTurnAcceleration) {
        this.maxStrafeAcceleration = maxStrafeAcceleration;
        turningAccelerationLimiter = new SlewRateLimiter(maxTurnAcceleration, -maxTurnAcceleration, 0);
        reset();
    }
    
    private double getTimeSecs () {
        return System.currentTimeMillis() / 1000.;
    }
    
    private double getAdjustedStrafeDeltaVelocityMagnitude (double requestedDeltaVelocityMagnitude) {
        double deltaTime = getTimeSecs() - lastApplicationTime;
        double maxDeltaVelocityMagnitude = maxStrafeAcceleration * deltaTime;
        
        return Transform.clamp(0, maxDeltaVelocityMagnitude).apply(requestedDeltaVelocityMagnitude);
    }
    
    public void reset () {
        lastApplicationTime = getTimeSecs();
        lastStrafeVelocity = new Vector<>(Nat.N2(), 0, 0);
        turningAccelerationLimiter.reset(0);
    }
    
    public ChassisSpeeds applyToSpeeds (ChassisSpeeds givenChassisSpeeds) {
        // Adjust strafe speeds
        Vector<N2> requestedStrafeVelocity = new Vector<>(Nat.N2(), givenChassisSpeeds.vxMetersPerSecond, givenChassisSpeeds.vyMetersPerSecond);
        Vector<N2> requestedDeltaSpeeds = requestedStrafeVelocity.subtract(lastStrafeVelocity);
        
        // Apply a scale to limit the magnitude of the delta velocity vector
        Vector<N2> adjustedDeltaSpeeds = requestedDeltaSpeeds.applyScale(this::getAdjustedStrafeDeltaVelocityMagnitude);
        
        // Calculate the adjusted strafe velocity
        // TODO: FIX CLAW VECTOR.ADD
        Vector<N2> adjustedStrafeVelocity = new Vector<N2>(
            Nat.N2(),
            lastStrafeVelocity.components[0] + adjustedDeltaSpeeds.components[0],
            lastStrafeVelocity.components[1] + adjustedDeltaSpeeds.components[1]
        );
        
        lastStrafeVelocity = adjustedStrafeVelocity;
        
        // Apply a limiter to the turning
        double adjustedTurnSpeed = turningAccelerationLimiter.calculate(givenChassisSpeeds.omegaRadiansPerSecond);
        
        // Update the lastApplicationTime
        lastApplicationTime = getTimeSecs();
        return new ChassisSpeeds(
            adjustedStrafeVelocity.components[0],
            adjustedStrafeVelocity.components[1],
            adjustedTurnSpeed
        );
    }
    
}
