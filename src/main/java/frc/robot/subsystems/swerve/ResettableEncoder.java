package frc.robot.subsystems.swerve;

import com.ctre.phoenix.sensors.CANCoder;

import claw.Setting;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * A wrapper around {@link CANCoder}
 */
public class ResettableEncoder {
    
    private final Setting<Double> offsetSetting;
    private final CANCoder encoder;
    private boolean inverted;
    
    public ResettableEncoder (Setting<Double> offsetSetting, int canId) {
        encoder = new CANCoder(canId);
        this.offsetSetting = offsetSetting;
    }
    
    // Say "g" = given rotation (what this ResettableEncoder returns), "a" = absolute (measured) rotation,
    // and "o" = offset.
    
    // So, if offset is defined such that g = a + o, if we want to reset the offset, o = g - a.
    
    public Rotation2d getRotation () {
        // g = a + o
        Rotation2d rotations = getAbsoluteRotation().plus(getOffset());
        return inverted
            ? Rotation2d.fromDegrees(-rotations.getDegrees())
            : rotations;
    }
    
    public void setRotation (Rotation2d rotation) {
        // o = g - a
        Rotation2d offset = rotation.minus(getAbsoluteRotation());
        offsetSetting.set(offset.getDegrees());
    }
    
    private Rotation2d getOffset () {
        return Rotation2d.fromDegrees(offsetSetting.get());
    }
    
    public void zeroRotation () {
        setRotation(Rotation2d.fromRotations(0));
    }
    
    private Rotation2d getAbsoluteRotation () {
        return Rotation2d.fromDegrees(encoder.getAbsolutePosition());
    }
    
    public void setInverted (boolean inverted) {
        this.inverted = inverted;
    }
    
}
