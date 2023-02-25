package frc.robot.commands;

import claw.logs.CLAWLogger;
import claw.math.Transform;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class IntakeEngagementCommand extends CommandBase {
    
    private static final double GRAVITY_FORCE_VOLTAGE = 1.2;
    
    private final PIDController pid = new PIDController(0.2, 0, 0);
    private final SlewRateLimiter limiter = new SlewRateLimiter(15, -15, 0);
    
    private final Intake intake;
    private final IntakeEngagement engagement;
    
    private final CLAWLogger LOG = CLAWLogger.getLogger("intake speed");
    
    public IntakeEngagementCommand (Intake intake, IntakeEngagement engagement) {
        this.intake = intake;
        this.engagement = engagement;
        addRequirements(intake);
    }
    
    private final Transform disengageIntakePositionToVoltage =
        // Scale from 2.4 at the very bottom to 0.7 at the top
        ((Transform)((double pos) -> pos * 2.4))
        .then(Transform.clamp(0.7, 2.4))
        
        // Negate speed because we're disengaging
        .then(Transform.NEGATE);
    
    private final Transform engageIntakePositionToVoltage =
        // Get a 0 for pos>0.2 and 1 for pos<0.2
        ((Transform)((double pos) -> 0.2 - pos))
        .then(Transform.SIGN)
        .then(Transform.clamp(0, 1))
        
        // Multiply by 1.2
        .then((Transform)((double x) -> 1.2*x));
    
    @Override
    public void initialize () {
        intake.stop();
        pid.reset();
        limiter.reset(0);
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
