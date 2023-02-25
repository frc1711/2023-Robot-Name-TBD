package frc.robot.commands;

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
    
    public IntakeEngagementCommand (Intake intake, IntakeEngagement engagement) {
        this.intake = intake;
        this.engagement = engagement;
        addRequirements(intake);
    }
    
    @Override
    public void initialize () {
        intake.stop();
        pid.reset();
        limiter.reset(0);
    }
    
    @Override
    public void execute () {
        // pid.calculate(intake.getEngagementVelocity(), engagement == IntakeEngagement.ENGAGE ? 80 : -80);
        intake.setEngagementVoltage(engagement == IntakeEngagement.ENGAGE ? 1.2 : -1.8);
        
    }
    
    @Override
    public void end (boolean interrupted) {
        intake.stop();
    }
    
    public enum IntakeEngagement {
        ENGAGE,
        DISENGAGE,
    }
    
}
