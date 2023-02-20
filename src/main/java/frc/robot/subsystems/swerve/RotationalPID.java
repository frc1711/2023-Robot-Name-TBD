package frc.robot.subsystems.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;

public class RotationalPID {
    
    private final PIDController pid;
    
    public RotationalPID (double kP, double kI, double kD, double setpointToleranceDegrees) {
        pid = new PIDController(kP, kI, kD);
        pid.setTolerance(setpointToleranceDegrees);
        pid.reset();
    }
    
    public double calculate (Rotation2d measured, Rotation2d setpoint) {
        double offsetDegrees = setpoint.getDegrees() - measured.getDegrees();
        double calculatedValue = pid.calculate(0, MathUtil.inputModulus(offsetDegrees, -180, 180));
        
        if (pid.atSetpoint()) return 0;
        else return calculatedValue;
    }
    
    public void reset () {
        pid.reset();
    }
    
}
