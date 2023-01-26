package frc.robot.subsystems.swerve;

import claw.CLAWLogger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;

public class RotationalPID {
    
    private final CLAWLogger log;
    private final PIDController pid;
    
    public RotationalPID (String name, double kP, double kI, double kD, double setpointToleranceDegrees) {
        this.log = CLAWLogger.getLogger(name);
        pid = new PIDController(kP, kI, kD);
        pid.setTolerance(setpointToleranceDegrees);
        pid.reset();
    }
    
    public double calculate (Rotation2d measured, Rotation2d setpoint) {
        double offsetDegrees = setpoint.getDegrees() - measured.getDegrees();
        log.sublog("offset").out(offsetDegrees+" deg");
        log.sublog("offsetWrap").out(MathUtil.inputModulus(offsetDegrees, -180, 180)+" deg");
        double calculatedValue = pid.calculate(0, MathUtil.inputModulus(offsetDegrees, -180, 180));
        
        if (pid.atSetpoint()) return 0;
        else return calculatedValue;
    }
    
    public void reset () {
        pid.reset();
    }
    
}
