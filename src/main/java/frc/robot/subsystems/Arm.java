// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import claw.CLAWRobot;
import claw.Setting;
import claw.hardware.Device;
import claw.logs.CLAWLogger;
import claw.math.InputTransform;
import claw.math.Transform;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LiveCommandTester;
import frc.robot.RobotContainer;

public class Arm extends SubsystemBase {
    
    /**
     * An offset in the claw's relative encoder's reading from the homing spot (fully collapsed) to the maximum
     * extension of the claw (fully released). This offset helps to prevent the claw from extending too far and
     * breaking itself.
     */
    private static final double CLAW_MAX_REACH_OFFSET = 24;
    
    /**
     * An offset in the claw's relative encoder's reading from the homing spot (fully collapsed) to the minimum
     * extension of the claw (almost fully grabbing). This offset helps to prevent the claw from contracting too
     * much and breaking itself.
     */
    private static final double CLAW_MIN_REACH_OFFSET = 0;
    
    // TODO: Add javadocs
    private static final double
        GRAB_OUTPUT_CURRENT = 20,
        HOME_OUTPUT_CURRENT = 13;
    
    private static final double
        CLAW_MOVE_VOLTAGE = 4,
        CLAW_HOMING_VOLTAGE = 1.8;
    
    private static final double
        ARM_MIN_ANGLE_DEGREES = -7,
        ARM_MAX_ANGLE_DEGREES = 99;
    
    private static final double ARM_CURRENT_LIMIT = 25;
    
    private static Arm armInstance;
    
    public static Arm getInstance () {
        if (armInstance == null) {
            armInstance = new Arm(
                new CANSparkMax(15, MotorType.kBrushless), 
                new CANSparkMax(14, MotorType.kBrushless)
            );
        }
        return armInstance;
    }
    
    private final CANSparkMax armMotor, clawMotor;
    
    private final Device<DutyCycle> armEncoder = new Device<>(
        "DIO.ENCODER.ARM.ARM_ENCODER",
        id -> {
            return new DutyCycle(new DigitalInput(id));
        },
        DutyCycle::close
    );
    
    private final Debouncer clawGrabDebouncer = new Debouncer(.3, DebounceType.kRising);
    private final Debouncer
        armCurrentStopFirstDebouncer = new Debouncer(0.23, DebounceType.kRising),
        armCurrentStopSecondDebouncer = new Debouncer(1.4, DebounceType.kFalling);
    
    private static final Setting<Double> ARM_ENCODER_ZERO = new Setting<>("ARM_ENCODER_CONFIG.ZERO", () -> 0.);
    private static final Setting<Double> ARM_ENCODER_NINETY = new Setting<>("ARM_ENCODER_CONFIG.NINETY", () -> 1.);
    
    private final Transform armDegreesOffsetToSpeed =
        ((Transform)(deg -> deg/8.))
        .then(Transform.clamp(-0.5, 0.5))
        .then(Transform.NEGATE);
    
    private double clawEncoderOffset = 0;
    
    private boolean clawHasBeenHomed = false;
    private boolean isHoldingObject = false;
    
    public Arm(CANSparkMax armMotor, CANSparkMax clawMotor) {
        this.armMotor = armMotor;
        armMotor.setIdleMode(IdleMode.kBrake);
        this.clawMotor = clawMotor;
        clawMotor.setIdleMode(IdleMode.kBrake);
        
        XboxController controller = new XboxController(2);
        Transform transform = InputTransform.getInputTransform(
            InputTransform.SQUARE_CURVE,
            0.2
        );
        
        LiveCommandTester tester = new LiveCommandTester(
            "Use controller 2. Left joystick to control the arm. " +
            "\n\nHold both triggers and press X to configure the arm down position. " +
            "Hold both triggers and press B to configure the arm up position.",
            liveValues -> {
                
                if (controller.getAButton() && !hasClawBeenHomed()) {
                    
                    // Run homing sequence
                    runClawHomingSequence();
                    
                } else if (hasClawBeenHomed() && controller.getRightBumper()) {
                    
                    // Grab claw
                    operateClaw(ClawMovement.GRAB);
                    
                } else if (hasClawBeenHomed() && controller.getLeftBumper()) {
                    
                    // Release claw
                    operateClaw(ClawMovement.RELEASE);
                    
                } else {
                    
                    // No claw operations
                    operateClaw(ClawMovement.NONE);
                    
                }
                
                if (controller.getLeftTriggerAxis() > 0.8 && controller.getRightTriggerAxis() > 0.8) {
                    if (controller.getXButton())
                        ARM_ENCODER_ZERO.set(armEncoder.get().getOutput());
                    else if (controller.getBButton())
                        ARM_ENCODER_NINETY.set(armEncoder.get().getOutput());
                }
                
                liveValues.setField("Claw can release", clawCanExtend());
                liveValues.setField("Claw can grab", clawCanContract());
                liveValues.setField("Output current", clawMotor.getAppliedOutput());
                liveValues.setField("Encoder reading", clawMotor.getEncoder().getPosition());
                liveValues.setField("Arm position", getArmRotation().getDegrees() + " deg");
                liveValues.setField("Arm current", armMotor.getOutputCurrent());
                
                if (controller.getYButton()) {
                    double armVoltage = transform.apply(controller.getLeftY());
                    setArmSpeedOverride(armVoltage);
                } else stopArm();
            },
            this::stop,
            this
        );
        
        CLAWRobot.getExtensibleCommandInterpreter().addCommandProcessor(
            tester.toCommandProcessor("armtest")
        );
        
        RobotContainer.putConfigSendable("Arm Subsystem", this);
    }
    
    public Rotation2d getArmRotation () {
        // xProp is the proportion from 0 to 90 degrees
        double xProp = (armEncoder.get().getOutput() - ARM_ENCODER_ZERO.get()) / (ARM_ENCODER_NINETY.get() - ARM_ENCODER_ZERO.get());
        return Rotation2d.fromDegrees(xProp * 90);
    }
    
    /**
     * Move the arm forward at a given speed on the interval [-1, 1],
     * ignoring the position of the arm (overriding safety stops).
     * @param input
     */
    public void setArmSpeedOverride (double input) {
        armMotor.setVoltage(input * 12);
    }
    
    public enum ArmPosition {
        HIGH    (Rotation2d.fromDegrees(95)),
        MIDDLE  (Rotation2d.fromDegrees(70)),
        LOW     (Rotation2d.fromDegrees(35));
        
        public final Rotation2d rotation;
        private ArmPosition (Rotation2d rotation) {
            this.rotation = rotation;
        }
    }
    
    public void setArmSpeed (double input) {
        
        double armDegrees = getArmRotation().getDegrees();
        boolean armCurrentLimitTripped = armCurrentStopSecondDebouncer.calculate(
            armCurrentStopFirstDebouncer.calculate(
                armMotor.getOutputCurrent() > ARM_CURRENT_LIMIT
            )
        );
        
        if (!armCurrentLimitTripped) {
            if ((input >= 0 && armDegrees < ARM_MAX_ANGLE_DEGREES) || (input < 0 && armDegrees > ARM_MIN_ANGLE_DEGREES)) {
                setArmSpeedOverride(input);
            } else stopArm();
        } else stopArm();
        
    }
    
    public void stopArm() {
        armMotor.setVoltage(0);
    }
    
    private final Debouncer homingSequenceDebouncer = new Debouncer(0.1, DebounceType.kRising);
    public void runClawHomingSequence () {
        if (homingSequenceDebouncer.calculate(clawMotor.getOutputCurrent() > HOME_OUTPUT_CURRENT)) {
            clawEncoderOffset = getClawEncoder();
            clawMotor.setVoltage(0);
            clawHasBeenHomed = true;
        } else {
            clawMotor.setVoltage(CLAW_HOMING_VOLTAGE);
        }
    }
    
    public boolean hasClawBeenHomed () {
        return clawHasBeenHomed;
    }
    
    private double getClawEncoder () {
        return -clawMotor.getEncoder().getPosition();
    }
    
    private boolean clawCanContract () {
        return getClawEncoder() - clawEncoderOffset > CLAW_MIN_REACH_OFFSET;
    }
    
    private boolean clawCanExtend () {
        return getClawEncoder() - clawEncoderOffset < CLAW_MAX_REACH_OFFSET;
    }
    
    public void operateClaw (ClawMovement move) {
        switch (move) {
            case NONE:
                clawMotor.stopMotor();
                break;
            case GRAB:
                if (isHoldingObject || !clawCanContract()) {
                    clawMotor.stopMotor();
                } else {
                    clawMotor.setVoltage(CLAW_MOVE_VOLTAGE);
                    isHoldingObject = clawGrabDebouncer.calculate(clawMotor.getOutputCurrent() > GRAB_OUTPUT_CURRENT);
                }
                break;
            case RELEASE:
                if (!clawCanExtend()) clawMotor.stopMotor();
                else clawMotor.setVoltage(-CLAW_MOVE_VOLTAGE);
                isHoldingObject = false;
                break;
        }
    }
    
    public enum ClawMovement {
        NONE,
        GRAB,
        RELEASE;
    }
    
    public void stop() {
        stopArm();
        operateClaw(ClawMovement.NONE);
    }
    
    @Override
    public void initSendable (SendableBuilder builder) {
        builder.addDoubleProperty("Claw output current", clawMotor::getOutputCurrent, null);
        builder.addDoubleProperty("Arm position", () -> getArmRotation().getDegrees(), null);
        builder.addDoubleProperty("Arm output current", () -> armMotor.getOutputCurrent(), null);
    }
    
}