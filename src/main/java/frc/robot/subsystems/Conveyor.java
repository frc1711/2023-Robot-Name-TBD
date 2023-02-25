package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import claw.hardware.Device;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Conveyor extends SubsystemBase{
	
	private static Conveyor conveyorInstance;
	public static Conveyor getInstance() {
		if (conveyorInstance == null) conveyorInstance = new Conveyor();
		return conveyorInstance;
	}

	private final Device<CANSparkMax> conveyorMotor = new Device<>(
		"CAN.MOTOR_CONTROLLER.CONVEYOR.CONVEYOR_MOTOR", 
		id -> {
			CANSparkMax motor = new CANSparkMax(id, MotorType.kBrushless);
			motor.setIdleMode(IdleMode.kBrake);
			return motor;
		},
		motor -> {
			motor.stopMotor();
			motor.close();
		}
	);
    
    public enum ConveyorMode {
        FORWARD (4),
        REVERSE (-4),
        STOP (0);
        
        private final double voltage;
        private ConveyorMode (double voltage) {
            this.voltage = voltage;
        }
    }
    
	public void setMode (ConveyorMode mode) {
		conveyorMotor.get().setVoltage(mode.voltage);
	}
    
}
