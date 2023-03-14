package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import claw.CLAWRobot;
import claw.rct.commands.CommandProcessor;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LiveCommandTester;

public class Claw extends SubsystemBase {
    
    /**
     * An offset in the claw's relative encoder's reading from the homing spot (fully collapsed) to the maximum
     * extension of the claw (fully released). This offset helps to prevent the claw from extending too far and
     * breaking itself.
     */
    private static final double CLAW_MAX_REACH_OFFSET = 39.78;
    
    // TODO: Add javadocs
    private static final double
        GRAB_OUTPUT_CURRENT = 20,
        HOME_OUTPUT_CURRENT = 20;
    
    private static final double
        CLAW_MOVE_VOLTAGE = 4,
        CLAW_HOMING_VOLTAGE = 1.8;
    
    private final CANSparkMax clawMotor = new CANSparkMax(14, MotorType.kBrushless);
    private final Debouncer clawGrabDebouncer = new Debouncer(.3, DebounceType.kRising);
    private final Debouncer homingSequenceDebouncer = new Debouncer(0.1, DebounceType.kRising);
    
    private double clawEncoderOffset = 0;
    private boolean hasBeenHomed = false;
    private boolean isHoldingObject = false;
    
    public Claw () {
        clawMotor.setIdleMode(IdleMode.kBrake);
        
        XboxController controller = new XboxController(3);
        CommandProcessor processor = new LiveCommandTester(
            "Use controller 3. A and B will move the claw in opposite directions. A is typically grab and B is typically release, " +
            "but it depends on the initial orientation of the claw. There are no protections on the claw's movement.",
            values -> {
                values.setField("Encoder reading", getClawEncoder() - clawEncoderOffset);
                
                if (controller.getAButton()) {
                    clawMotor.setVoltage(CLAW_MOVE_VOLTAGE);
                } else if (controller.getBButton()) {
                    clawMotor.setVoltage(-CLAW_MOVE_VOLTAGE);
                } else {
                    clawMotor.stopMotor();
                }
            },
            () -> operateClaw(ClawMovement.NONE),
            this
        ).toCommandProcessor("clawtest");
        
        CLAWRobot.getExtensibleCommandInterpreter().addCommandProcessor(processor);
        
    }
    
    public void runClawHomingSequence () {
        if (homingSequenceDebouncer.calculate(clawMotor.getOutputCurrent() > HOME_OUTPUT_CURRENT)) {
            clawEncoderOffset = getClawEncoder();
            clawMotor.setVoltage(0);
            hasBeenHomed = true;
        } else {
            clawMotor.setVoltage(CLAW_HOMING_VOLTAGE);
        }
    }
    
    public void homeAsFullyOpen () {
        clawEncoderOffset = getClawEncoder() - CLAW_MAX_REACH_OFFSET;
        hasBeenHomed = true;
    }
    
    public boolean hasBeenHomed () {
        return hasBeenHomed;
    }
    
    /**
     * A greater encoder value indicates that the claw is more open
     */
    private double getClawEncoder () {
        return -clawMotor.getEncoder().getPosition();
    }
    
    private boolean isClawOverLowerLimit () {
        return getClawEncoder() - clawEncoderOffset > 0;
    }
    
    private boolean isClawUnderUpperLimit () {
        return getClawEncoder() - clawEncoderOffset < CLAW_MAX_REACH_OFFSET;
    }
    
    public boolean isFullyReleased () {
        return !isClawUnderUpperLimit();
    }
    
    public boolean isFullyGrabbing () {
        return isHoldingObject || !isClawOverLowerLimit();
    }
    
    public void operateClaw (ClawMovement move) {
        switch (move) {
            case NONE:
                clawMotor.stopMotor();
                break;
            case GRAB:
                
                // TODO: Make it so that, once isHoldingObject is first set to true, there is an "object hold position"
                // which is stored so that the claw will actively drive to this position in case the grip starts to loosen.
                // This would prevent the issue of the grip loosening over time, which happens now because the motor stops driving
                // entirely once it first detects that it's grabbing an object with isHoldingObject
                
                if (isFullyGrabbing()) {
                    clawMotor.stopMotor();
                } else {
                    clawMotor.setVoltage(CLAW_MOVE_VOLTAGE);
                    isHoldingObject = clawGrabDebouncer.calculate(clawMotor.getOutputCurrent() > GRAB_OUTPUT_CURRENT);
                }
                break;
            case RELEASE:
                if (isFullyReleased())
                    clawMotor.stopMotor();
                else
                    clawMotor.setVoltage(-CLAW_MOVE_VOLTAGE);
                isHoldingObject = false;
                break;
        }
    }
    
    public enum ClawMovement {
        NONE,
        GRAB,
        RELEASE;
    }
    
    @Override
    public void initSendable (SendableBuilder builder) {
        builder.addDoubleProperty("Claw output current", clawMotor::getOutputCurrent, null);
    }
    
}
