package frc.robot.commands.auton;

import claw.api.CLAWLogger;

public class AutobalanceFeedforward {
    
    private static final CLAWLogger LOG = CLAWLogger.getLogger("autobalance");
    
    private final SpeedCalculator baseSpeedCalculator;
    private final SpeedCalculator momentumCancel;
    
    private double lastAngleInput = 0;
    private long lastCalculationTime = 0;
    private boolean resetMomentum = true;
    
    private double setpointTolerance = 0;
    
    public AutobalanceFeedforward (SpeedCalculator baseSpeedCalculator, SpeedCalculator momentumCancel) {
        this.baseSpeedCalculator = baseSpeedCalculator;
        this.momentumCancel = momentumCancel;
    }
    
    public void reset () {
        resetMomentum = true;
    }
    
    public void setTolerance (double tolerance) {
        setpointTolerance = tolerance;
    }
    
    public double calculate (double angleInput) {
        long currentTime = System.currentTimeMillis();
        double momentumCancelSpeed = resetMomentum
            ? 0
            : momentumCancel.calculate((angleInput - lastAngleInput) * (currentTime - lastCalculationTime));
        resetMomentum = false;
        
        LOG.sublog("momentumCancel").out(momentumCancelSpeed+"");
        
        lastAngleInput = angleInput;
        lastCalculationTime = currentTime;
        
        double baseSpeed = baseSpeedCalculator.calculate(angleInput);
        
        LOG.sublog("baseSpeed").out(baseSpeed+"");
        
        return Math.abs(angleInput) > setpointTolerance
            ? baseSpeed + momentumCancelSpeed
            : 0;
    }
    
    public static interface SpeedCalculator {
        public double calculate (double input);
    }
    
}
