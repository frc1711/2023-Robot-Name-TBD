package frc.robot.subsystems.swerve;

import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * A wrapper around {@link CANCoder}
 */
public class ResettableEncoder {
    
    private static final double OFFSET_PARAM_TO_ROTATION = 0.01;
    
    private final CANCoder encoder;
    private Rotation2d offset;
    
    public ResettableEncoder (int canId) {
        encoder = new CANCoder(canId);
        offset = getOffsetConfig();
    }
    
    // Say "g" = given rotation (what this ResettableEncoder returns), "a" = absolute (measured) rotation,
    // and "o" = offset.
    
    // So, if offset is defined such that g = a + o, if we want to reset the offset, o = g - a.
    
    public Rotation2d getRotation () {
        // g = a + o
        return getAbsoluteRotation().plus(offset);
    }
    
    public void setRotation (Rotation2d rotation) {
        // o = g - a
        offset = getAbsoluteRotation().plus(rotation);
        setOffsetConfig(offset);
    }
    
    public void zeroRotation () {
        setRotation(Rotation2d.fromRotations(0));
    }
    
    private Rotation2d getOffsetConfig () {
        return Rotation2d.fromDegrees(encoder.configGetCustomParam(0) * OFFSET_PARAM_TO_ROTATION);
    }
    
    private void setOffsetConfig (Rotation2d newOffset) {
        encoder.configSetCustomParam((int)(newOffset.getDegrees() / OFFSET_PARAM_TO_ROTATION), 0);
    }
    
    private Rotation2d getAbsoluteRotation () {
        return Rotation2d.fromDegrees(encoder.getAbsolutePosition());
    }
    
}
