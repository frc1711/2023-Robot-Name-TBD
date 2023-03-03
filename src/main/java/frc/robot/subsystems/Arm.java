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
import claw.math.InputTransform;
import claw.math.Transform;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LiveCommandTester;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DigitalInputEncoder.AnglePoint;

public class Arm extends SubsystemBase {
    
    private static final double
        ARM_MIN_ANGLE_DEGREES = -7,
        ARM_MAX_ANGLE_DEGREES = 99;
    
    private static final double ARM_CURRENT_LIMIT = 25;
    
    private static Arm armInstance;
    
    public static Arm getInstance () {
        if (armInstance == null) {
            armInstance = new Arm(
                new CANSparkMax(15, MotorType.kBrushless)
            );
        }
        return armInstance;
    }
    
    private final CANSparkMax armMotor;
    
    private static final Setting<Double> ARM_ENCODER_ZERO = new Setting<>("ARM_ENCODER_CONFIG.ZERO", () -> 0.);
    private static final Setting<Double> ARM_ENCODER_NINETY = new Setting<>("ARM_ENCODER_CONFIG.NINETY", () -> 1.);
    
    private final Device<DigitalInputEncoder> armEncoder = new Device<>(
        "DIO.ENCODER.ARM.ARM_ENCODER",
        id -> {
            return new DigitalInputEncoder(
                new DutyCycle(new DigitalInput(2)),
                false,
                new AnglePoint(0, ARM_ENCODER_ZERO.get()),
                new AnglePoint(90, ARM_ENCODER_NINETY.get())
            );
        },
        DigitalInputEncoder::close
    );
    
    private final Debouncer
        armCurrentStopFirstDebouncer = new Debouncer(0.23, DebounceType.kRising),
        armCurrentStopSecondDebouncer = new Debouncer(1.4, DebounceType.kFalling);
    
    private final Transform armDegreesOffsetToSpeed =
        ((Transform)(deg -> deg/8.))
        .then(Transform.clamp(-0.5, 0.5))
        .then(Transform.NEGATE);
    
    public Arm(CANSparkMax armMotor) {
        this.armMotor = armMotor;
        armMotor.setIdleMode(IdleMode.kBrake);
        
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
                
                if (controller.getLeftTriggerAxis() > 0.8 && controller.getRightTriggerAxis() > 0.8) {
                    if (controller.getXButton())
                        ARM_ENCODER_ZERO.set(armEncoder.get().getRawDutyCycleValue());
                    else if (controller.getBButton())
                        ARM_ENCODER_NINETY.set(armEncoder.get().getRawDutyCycleValue());
                }
                
                liveValues.setField("Arm position", getArmRotation().getDegrees() + " deg");
                liveValues.setField("Arm current", armMotor.getOutputCurrent());
                
                liveValues.setField("Arm encoder duty cycle input", armEncoder.get().getRawDutyCycleValue());
                
                if (controller.getYButton()) {
                    double armVoltage = transform.apply(controller.getLeftY());
                    setArmSpeedOverride(armVoltage);
                } else stop();
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
        return armEncoder.get().getRotation();
        // // xProp is the proportion from 0 to 90 degrees
        // double xProp = (armEncoder.get().getOutput() - ARM_ENCODER_ZERO.get()) / (ARM_ENCODER_NINETY.get() - ARM_ENCODER_ZERO.get());
        // return Rotation2d.fromDegrees(xProp * 90);
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
        LOW     (Rotation2d.fromDegrees(35)),
        STOWED  (Rotation2d.fromDegrees(-6.5));
        
        public final Rotation2d rotation;
        private ArmPosition (Rotation2d rotation) {
            this.rotation = rotation;
        }
    }
    
    public void moveArmToPosition (ArmPosition position) {
        moveArmToRotation(position.rotation);
    }
    
    public void moveArmToRotation (Rotation2d rotation) {
        double degreesOffset = getArmRotation().minus(rotation).getDegrees();
        setArmSpeed(armDegreesOffsetToSpeed.apply(degreesOffset));
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
            } else stop();
        } else stop();
        
    }
    
    public void stop () {
        armMotor.stopMotor();
    }
    
    @Override
    public void initSendable (SendableBuilder builder) {
        builder.addDoubleProperty("Arm position", () -> getArmRotation().getDegrees(), null);
        builder.addDoubleProperty("Arm output current", () -> armMotor.getOutputCurrent(), null);
    }
    
}