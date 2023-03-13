package frc.robot.commands.auton;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.Swerve;

/**
 * A very primitive, fallback command for driving swerve with given x and y speeds for a set amount of time,
 * with a given speed ramp rate.
 */
public class TimedDriveCommand extends CommandBase {
    
    private final Swerve swerve;
    
    private final SlewRateLimiter xLimiter, yLimiter;
    private final double givenXSpeed, givenYSpeed;
    
    private final double totalDuration, durationBeforeRampDown;
    private double startTime = 0;
    
    /**
     * Create a new timed drive command for the swerve drive which drives it at the given speeds for a provided duration.
     * The speeds also ramp up at the given rampRate, measured in units/second where the provided units are the measurement
     * of speed for x and y. +x is forward, +y is to the left
     */
    public TimedDriveCommand (Swerve swerve, double xSpeed, double ySpeed, double rampRate, double duration) {
        this.swerve = swerve;
        givenXSpeed = xSpeed;
        givenYSpeed = ySpeed;
        
        double maxDirectionalSpeed = Math.max(Math.abs(xSpeed), Math.abs(ySpeed));
        double rampDownDuration = maxDirectionalSpeed / rampRate;
        
        totalDuration = duration;
        durationBeforeRampDown = duration - rampDownDuration;
        
        this.xLimiter = new SlewRateLimiter(rampRate, -rampRate, 0);
        this.yLimiter = new SlewRateLimiter(rampRate, -rampRate, 0);
        
        addRequirements(swerve);
    }
    
    @Override
    public void initialize () {
        startTime = getCurrentTime();
        
    }
    
    private double getCurrentTime () {
        return System.currentTimeMillis() / 1000.;
    }
    
    private double getRampedSpeed (SlewRateLimiter speedLimiter, double givenSpeed) {
        double preRampSpeed = (getCurrentTime() < startTime + durationBeforeRampDown) ? givenSpeed : 0;
        return speedLimiter.calculate(preRampSpeed);
    }
    
    @Override
    public void execute () {
        double xSpeed = getRampedSpeed(xLimiter, givenXSpeed);
        double ySpeed = getRampedSpeed(yLimiter, givenYSpeed);
        swerve.moveRobotRelative(new ChassisSpeeds(xSpeed, ySpeed, 0));
    }
    
    @Override
    public void end (boolean interrupted) {
        swerve.stop();
    }
    
    @Override
    public boolean isFinished () {
        return getCurrentTime() > startTime + totalDuration;
    }
    
}
