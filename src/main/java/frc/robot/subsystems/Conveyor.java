package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Conveyor extends SubsystemBase{
	
	private static Conveyor conveyorInstance;
	public static Conveyor getInstance() {
		if (conveyorInstance == null) conveyorInstance = new Conveyor();
		return conveyorInstance;
	}

	private final CANSparkMax conveyorMotor = new CANSparkMax(13, MotorType.kBrushless);
    
    public Conveyor () {
        conveyorMotor.setIdleMode(IdleMode.kBrake);
    }
    
    public enum ConveyorMode {
        FORWARD (4),
        FAST_FORWARD (8),
        REVERSE (-4),
        FAST_REVERSE (-8),
        STOP (0);
        
        private final double voltage;
        private ConveyorMode (double voltage) {
            this.voltage = voltage;
        }
    }
    
	public void setMode (ConveyorMode mode) {
		conveyorMotor.setVoltage(mode.voltage);
	}
    
}
