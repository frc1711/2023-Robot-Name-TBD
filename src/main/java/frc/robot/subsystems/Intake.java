// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import claw.CLAWRobot;
import claw.hardware.Device;
import claw.hardware.LimitSwitchDevice;
import claw.hardware.Device.DeviceInitializer;
import claw.hardware.LimitSwitchDevice.NormalState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LiveCommandTester;
import frc.robot.RobotContainer;

public class Intake extends SubsystemBase {
    
    private static final double
        ENGAGE_VOLTAGE = 1.2,
        DISENGAGE_VOLTAGE = -1.5;
    
    private static final double INTAKE_FORWARD_SPEED = 3;
    
    private static Intake intakeInstance;
    
    public static Intake getInstance() {
        if (intakeInstance == null)
            intakeInstance = new Intake();
        return intakeInstance;
    }
    
    private static DeviceInitializer<CANSparkMax> getSparkMaxInitializer (IdleMode idleMode) {
        return (id) -> {
            CANSparkMax motor = new CANSparkMax(id, MotorType.kBrushless);
            motor.clearFaults();
            motor.setIdleMode(idleMode);
            return motor;
        };
    }
    
    private static void finalizeSparkMax (CANSparkMax motor) {
        motor.stopMotor();
        motor.close();
    }
    
    private final Device<CANSparkMax>
        topRoller = new Device<>(
            "CAN.MOTOR_CONTROLLER.INTAKE.TOP_ROLLER",
            getSparkMaxInitializer(IdleMode.kCoast),
            Intake::finalizeSparkMax
        ),
        bottomRoller = new Device<>(
            "CAN.MOTOR_CONTROLLER.INTAKE.BOTTOM_ROLLER",
            getSparkMaxInitializer(IdleMode.kCoast),
            Intake::finalizeSparkMax
        ),
        leftEngage = new Device<>(
            "CAN.MOTOR_CONTROLLER.INTAKE.LEFT_ENGAGE",
            getSparkMaxInitializer(IdleMode.kBrake),
            Intake::finalizeSparkMax
        ),
        rightEngage = new Device<>(
            "CAN.MOTOR_CONTROLLER.INTAKE.RIGHT_ENGAGE",
            getSparkMaxInitializer(IdleMode.kBrake),
            Intake::finalizeSparkMax
        );
    
    private final LimitSwitchDevice
        lowerLimitSwitch = new LimitSwitchDevice("DIO.LIMIT_SWITCH.INTAKE.LOWER_LIMIT", NormalState.NORMALLY_CLOSED),
        upperLimitSwitch = new LimitSwitchDevice("DIO.LIMIT_SWITCH.INTAKE.UPPER_LIMIT", NormalState.NORMALLY_CLOSED);
    
    private boolean isFullyEngaged = false;
    
    public Intake () {
        RobotContainer.putConfigSendable("IntakeSubsystem", this);
        
        XboxController controller = new XboxController(0);
        LiveCommandTester<XboxController> tester = new LiveCommandTester<>(
            () -> controller,
            c -> {
                if (c.getAButton()) {
                    setIntakeEngagement(IntakeEngagement.ENGAGE);
                } else setIntakeEngagement(IntakeEngagement.DISENGAGE);
            },
            this::stop,
            this
        );
        
        CLAWRobot.getExtensibleCommandInterpreter().addCommandProcessor(tester.toCommandProcessor("intaketest"));
        
        RobotContainer.putConfigSendable("Intake Subsystem", this);
    }
    
    public enum IntakeEngagement {
        ENGAGE,
        DISENGAGE,
        PASSIVE;
    }
    
    public void setIntakeEngagement (IntakeEngagement engagement) {
        if (engagement == IntakeEngagement.DISENGAGE) {
            isFullyEngaged = false;
        } else if (isLowerPressed()) {
            isFullyEngaged = true;
        }
        
        double engageSpeed = 0;
        
        if (engagement == IntakeEngagement.ENGAGE) {
            if (!isLowerPressed()) {
                engageSpeed = ENGAGE_VOLTAGE;
            }
        } else if (engagement == IntakeEngagement.DISENGAGE) {
            if (!isUpperPressed()) {
                engageSpeed = DISENGAGE_VOLTAGE;
            }
        }
        
        leftEngage.get().setVoltage(engageSpeed);
        rightEngage.get().setVoltage(-engageSpeed);
    }
    
    public void initSendable (SendableBuilder builder) {
        builder.addBooleanProperty("lower-limit", this::isLowerPressed, null);
        builder.addBooleanProperty("upper-limit", this::isUpperPressed, null);
    }
    
    private boolean isLowerPressed () {
        return !lowerLimitSwitch.isPressed();
    }
    
    private boolean isUpperPressed () {
        return !upperLimitSwitch.isPressed();
    }
    
    public enum IntakeMode {
        FORWARD (1),
        REVERSE (-1),
        STOP (0);
        
        private final double speedMult;
        private IntakeMode (double speedMult) {
            this.speedMult = speedMult;
        }
    }
    
    public void setIntakeMode (IntakeMode mode) {
        double speed = mode.speedMult * INTAKE_FORWARD_SPEED;
        topRoller.get().setVoltage(speed);
        bottomRoller.get().setVoltage(speed);
    }
    
    public void stop () {
        setIntakeEngagement(IntakeEngagement.PASSIVE);
        setIntakeMode(IntakeMode.STOP);
    }
    
}