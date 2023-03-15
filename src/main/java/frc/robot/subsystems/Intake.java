// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import claw.CLAWRobot;
import claw.hardware.LimitSwitchDevice;
import claw.hardware.LimitSwitchDevice.NormalState;
import claw.math.input.InputTransform;
import claw.math.Transform;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LiveCommandTester;
import frc.robot.RobotContainer;

public class Intake extends SubsystemBase {
    
    private static Intake intakeInstance;
    
    public static Intake getInstance() {
        if (intakeInstance == null)
            intakeInstance = new Intake();
        return intakeInstance;
    }
    
    private static CANSparkMax getSparkMax (int id, IdleMode idleMode) {
        CANSparkMax motor = new CANSparkMax(id, MotorType.kBrushless);
        motor.clearFaults();
        motor.setIdleMode(idleMode);
        return motor;
    }
    
    private final CANSparkMax
        topRoller = getSparkMax(16, IdleMode.kCoast),
        bottomRoller = getSparkMax(17, IdleMode.kCoast),
        leftEngage = getSparkMax(19, IdleMode.kBrake),
        rightEngage = getSparkMax(18, IdleMode.kBrake);
    
    private final LimitSwitchDevice
        lowerLimitSwitch = new LimitSwitchDevice(new DigitalInput(1), NormalState.NORMALLY_CLOSED),
        upperLimitSwitch = new LimitSwitchDevice(new DigitalInput(0), NormalState.NORMALLY_CLOSED);
    
    private final double BOTTOM_INTAKE_RAW_POSITION = 6.69;
    private double engagementPositionOffset = getEngagementRawPosition();
    
    public Intake () {
        RobotContainer.putConfigSendable("Intake Subsystem", this);
        LiveCommandTester tester = new LiveCommandTester(
            "Use controller 1.",
            liveFields -> {
                liveFields.setField("Intake position", getEngagementPosition());
            },
            this::stop,
            this
        );
        
        CLAWRobot.getExtensibleCommandInterpreter().addCommandProcessor(tester.toCommandProcessor("intaketest"));
    }
    
    public enum IntakeSpeedMode {
        CONE            (-3.8, -4.5),
        CONE_REVERSE    (3.8, 4.5),
        
        CUBE            (-2.3, -2.8),
        CUBE_REVERSE    (2.3, 2.8),
        
        STOP            (0, 0);
        
        private final double intakeTopVoltage, intakeBottomVoltage;
        private IntakeSpeedMode (double intakeTopVoltage, double intakeBottomVoltage) {
            this.intakeTopVoltage = intakeTopVoltage;
            this.intakeBottomVoltage = intakeBottomVoltage;
        }
    }
    
    public enum IntakeEngagement {
        ENGAGE (
            // Get a 0 for pos>0.2 and 1 for pos<0.25
            ((Transform)((double pos) -> 0.25 - pos))
            .then(Transform.SIGN)
            .then(Transform.clamp(0, 1))
            
            // Multiply for the final voltage
            .then((Transform)((double x) -> 1.5*x))
        ),
        
        DISENGAGE (
            // Apply a curve so the engagement voltage isn't linear
            // and it sharply drops off at the beginning
            InputTransform.THREE_HALVES_CURVE
            
            // Scale from 3.8 at the very bottom to 0.5 at the top
            .then((Transform)((double pos) -> pos * 3.8))
            .then(Transform.clamp(1.2, 3.8))
            
            // Negate speed because we're disengaging
            .then(Transform.NEGATE)
        ),
        
        PASSIVE (Transform.clamp(0, 0));
        
        private final Transform positionToVoltage;
        private IntakeEngagement (Transform positionToVoltage) {
            this.positionToVoltage = positionToVoltage;
        }
    }
    
    public void setIntakeSpeedMode (IntakeSpeedMode mode) {
        topRoller.setVoltage(mode.intakeTopVoltage);
        bottomRoller.setVoltage(mode.intakeBottomVoltage);
    }
    
    public void setIntakeEngagement (IntakeEngagement engagement) {
        double voltage = engagement.positionToVoltage.apply(getEngagementPosition());
        
        if (engagement == IntakeEngagement.ENGAGE) {
            setEngagementVoltage(lowerLimitSwitch.isPressed() ? 0 : voltage);
        } else if (engagement == IntakeEngagement.DISENGAGE) {
            setEngagementVoltage(upperLimitSwitch.isPressed() ? 0 : voltage);
        } else if (engagement == IntakeEngagement.PASSIVE) {
            setEngagementVoltage(voltage);
        }
    }
    
    /**
     * The raw engagement position according to the encoders.
     */
    private double getEngagementRawPosition () {
        return leftEngage.getEncoder().getPosition() - rightEngage.getEncoder().getPosition();
    }
    
    /**
     * Get the position of the intake engagement, adjusted so that 0 represents fully disengaged and 1 fully engaged.
     */
    public double getEngagementPosition () {
        return (getEngagementRawPosition() - engagementPositionOffset) / BOTTOM_INTAKE_RAW_POSITION;
    }
    
    /**
     * Set the voltage of both the left and right engagement motor controllers
     * so that a positive value is engaging and a negative value is disengaging.
     */
    private void setEngagementVoltage (double voltage) {
        leftEngage.setVoltage(voltage);
        rightEngage.setVoltage(-voltage);
    }
    
    private void stopEngagementMotors () {
        leftEngage.stopMotor();
        rightEngage.stopMotor();
    }
    
    public void stop () {
        stopEngagementMotors();
        setIntakeSpeedMode(IntakeSpeedMode.STOP);
    }
    
    public void initSendable (SendableBuilder builder) {
        builder.addBooleanProperty("lower-limit", lowerLimitSwitch::isPressed, null);
        builder.addBooleanProperty("upper-limit", upperLimitSwitch::isPressed, null);
    }
    
    @Override
    public void periodic () {
        if (upperLimitSwitch.isPressed() && !lowerLimitSwitch.isPressed())
            engagementPositionOffset = getEngagementRawPosition();
        else if (lowerLimitSwitch.isPressed() && !upperLimitSwitch.isPressed())
            engagementPositionOffset = getEngagementRawPosition() - BOTTOM_INTAKE_RAW_POSITION;
    }
    
}