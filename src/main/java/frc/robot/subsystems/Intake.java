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
import claw.math.InputTransform;
import claw.math.Transform;
import edu.wpi.first.util.sendable.SendableBuilder;
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
    
    private boolean isLowerPressed () {
        return !lowerLimitSwitch.isPressed();
    }
    
    private boolean isUpperPressed () {
        return !upperLimitSwitch.isPressed();
    }
    
    public enum IntakeSpeedMode {
        CONE            (-2.3, -2.8),
        CONE_REVERSE    (2.3, 2.8),
        
        CUBE            (-3.8, -4.5),
        CUBE_REVERSE    (3.8, 4.5),
        
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
            .then(Transform.clamp(0.75, 3.8))
            
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
        topRoller.get().setVoltage(mode.intakeTopVoltage);
        bottomRoller.get().setVoltage(mode.intakeBottomVoltage);
    }
    
    public void setIntakeEngagement (IntakeEngagement engagement) {
        double voltage = engagement.positionToVoltage.apply(getEngagementPosition());
        
        if (engagement == IntakeEngagement.ENGAGE) {
            setEngagementVoltage(isLowerPressed() ? 0 : voltage);
        } else if (engagement == IntakeEngagement.DISENGAGE) {
            setEngagementVoltage(isUpperPressed() ? 0 : voltage);
        } else if (engagement == IntakeEngagement.PASSIVE) {
            setEngagementVoltage(voltage);
        }
    }
    
    /**
     * The raw engagement position according to the encoders.
     */
    private double getEngagementRawPosition () {
        return leftEngage.get().getEncoder().getPosition() - rightEngage.get().getEncoder().getPosition();
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
        leftEngage.get().setVoltage(voltage);
        rightEngage.get().setVoltage(-voltage);
    }
    
    private void stopEngagementMotors () {
        leftEngage.get().stopMotor();
        rightEngage.get().stopMotor();
    }
    
    public void stop () {
        stopEngagementMotors();
        setIntakeSpeedMode(IntakeSpeedMode.STOP);
    }
    
    public void initSendable (SendableBuilder builder) {
        builder.addBooleanProperty("lower-limit", this::isLowerPressed, null);
        builder.addBooleanProperty("upper-limit", this::isUpperPressed, null);
    }
    
    @Override
    public void periodic () {
        if (isUpperPressed() && !isLowerPressed())
            engagementPositionOffset = getEngagementRawPosition();
        else if (isLowerPressed() && !isUpperPressed())
            engagementPositionOffset = getEngagementRawPosition() - BOTTOM_INTAKE_RAW_POSITION;
    }
    
}