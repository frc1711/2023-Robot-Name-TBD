package frc.robot.commands;

import claw.math.InputTransform;
import claw.math.Transform;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class IntakeEngagementCommand extends CommandBase {
    
    private final Intake intake;
    private final IntakeEngagement engagement;
    
    public IntakeEngagementCommand (Intake intake, IntakeEngagement engagement) {
        this.intake = intake;
        this.engagement = engagement;
        addRequirements(intake);
    }
    
    private final Transform disengageIntakePositionToVoltage =
        // Apply a curve so the engagement voltage isn't linear
        // and it sharply drops off at the beginning
        InputTransform.THREE_HALVES_CURVE
        
        // Scale from 3.8 at the very bottom to 0.5 at the top
        .then((Transform)((double pos) -> pos * 3.8))
        .then(Transform.clamp(0.75, 3.8))
        
        // Negate speed because we're disengaging
        .then(Transform.NEGATE);
    
    private final Transform engageIntakePositionToVoltage =
        // Get a 0 for pos>0.2 and 1 for pos<0.25
        ((Transform)((double pos) -> 0.25 - pos))
        .then(Transform.SIGN)
        .then(Transform.clamp(0, 1))
        
        // Multiply for the final voltage
        .then((Transform)((double x) -> 1.5*x));
    
    @Override
    public void initialize () {
        intake.stop();
    }
    
    @Override
    public void execute () {
        if (engagement == IntakeEngagement.DISENGAGE) {
            // Disengaging
            intake.setEngagementVoltage(disengageIntakePositionToVoltage.apply(intake.getEngagementPosition()));
        } else {
            // Engaging
            intake.setEngagementVoltage(engageIntakePositionToVoltage.apply(intake.getEngagementPosition()));
        }
    }
    
    @Override
    public void end (boolean interrupted) {
        intake.stop();
    }
    
    @Override
    public boolean isFinished () {
        if (engagement == IntakeEngagement.DISENGAGE) {
            return intake.isUpperPressed();
        } else {
            return intake.isLowerPressed();
        }
    }
    
    public enum IntakeEngagement {
        ENGAGE,
        DISENGAGE,
    }
    
}
