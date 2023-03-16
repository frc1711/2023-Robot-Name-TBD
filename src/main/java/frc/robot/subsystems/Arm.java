// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import claw.CLAWRobot;
import claw.Setting;
import claw.math.input.InputTransform;
import claw.math.Transform;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
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
import frc.robot.subsystems.DigitalInputEncoder.AnglePoint;
import frc.robot.vision.VisionManager;

public class Arm extends SubsystemBase {
    
    public static final double
        ARM_MIN_ANGLE_DEGREES = -4.5,
        ARM_MAX_ANGLE_DEGREES = 99;
    
    private static final double ARM_CURRENT_LIMIT = 25;
    
    private static Arm armInstance;
    
    public static Arm getInstance () {
        if (armInstance == null) {
            armInstance = new Arm();
        }
        return armInstance;
    }
    
    private final CANSparkMax
        leftArmMotor = new CANSparkMax(15, MotorType.kBrushless),
        rightArmMotor = new CANSparkMax(20, MotorType.kBrushless);
    
    private static final Setting<Double> ARM_ENCODER_ZERO = new Setting<>("ARM_ENCODER_CONFIG.ZERO", () -> 0.);
    private static final Setting<Double> ARM_ENCODER_NINETY = new Setting<>("ARM_ENCODER_CONFIG.NINETY", () -> 1.);
    
    private final DigitalInputEncoder armEncoder = new DigitalInputEncoder(
        new DutyCycle(new DigitalInput(2)),
        false,
        new AnglePoint(0, ARM_ENCODER_ZERO.get()),
        new AnglePoint(90, ARM_ENCODER_NINETY.get())
    );
    
    private final Debouncer
        armCurrentStopFirstDebouncer = new Debouncer(0.23, DebounceType.kRising),
        armCurrentStopSecondDebouncer = new Debouncer(1.4, DebounceType.kFalling);
    
    public Arm () {
        leftArmMotor.setIdleMode(IdleMode.kBrake);
        rightArmMotor.setIdleMode(IdleMode.kBrake);
        
        XboxController controller = new XboxController(2);
        Transform transform = new InputTransform(
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
                        ARM_ENCODER_ZERO.set(armEncoder.getRawDutyCycleValue());
                    else if (controller.getBButton())
                        ARM_ENCODER_NINETY.set(armEncoder.getRawDutyCycleValue());
                }
                
                liveValues.setField("Arm position", getArmRotation().getDegrees() + " deg");
                liveValues.setField("Left arm current", leftArmMotor.getOutputCurrent());
                liveValues.setField("Right arm current", rightArmMotor.getOutputCurrent());
                
                liveValues.setField("Arm encoder duty cycle input", armEncoder.getRawDutyCycleValue());
                
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
        return armEncoder.getRotation();
        // // xProp is the proportion from 0 to 90 degrees
        // double xProp = (armEncoder.get().getOutput() - ARM_ENCODER_ZERO.get()) / (ARM_ENCODER_NINETY.get() - ARM_ENCODER_ZERO.get());
        // return Rotation2d.fromDegrees(xProp * 90);
    }
    
    private final SlewRateLimiter armSpeedFilter = new SlewRateLimiter(1.5, -1.5, 0);
    private final SimpleMotorFeedforward armSpeedFeedforward = new SimpleMotorFeedforward(0.05, 0.95);
    private static final double ARM_GRAVITY_ACCEL = 0.025;
    
    private final Transform armSpeedToVoltage =
        Transform.clamp(-1, 1)
        .then(armSpeedFilter::calculate)
        .then(V -> armSpeedFeedforward.calculate(V) + ARM_GRAVITY_ACCEL * getArmRotation().getSin())
        .then(speed -> speed*7);
    
    /**
     * Move the arm forward at a given speed on the interval [-1, 1],
     * ignoring the position of the arm (overriding safety stops).
     * @param input
     */
    public void setArmSpeedOverride (double input) {
        double armVoltage = armSpeedToVoltage.apply(input);
        
        leftArmMotor.setVoltage(armVoltage);
        rightArmMotor.setVoltage(-armVoltage);
    }
    
    private final Transform degreesOffsetToMovement =
        ((Transform)(deg -> Math.abs(deg) > 2 ? deg : 0))
        .then(deg -> deg/25)
        .then(InputTransform.THREE_HALVES_CURVE)
        .then(Transform.clamp(-1, 1))
        .then(speed -> speed*0.6)
        .then(Transform.NEGATE);
    
    public double getSpeedToMoveToRotation (Rotation2d targetRotation) {
        return degreesOffsetToMovement.apply(getArmRotation().minus(targetRotation).getDegrees());
    }
    
    public enum ArmPosition {
        HIGH    (Rotation2d.fromDegrees(95)),
        MIDDLE  (Rotation2d.fromDegrees(70)),
        LOW     (Rotation2d.fromDegrees(35)),
        STOWED  (Rotation2d.fromDegrees(-6));
        
        public final Rotation2d rotation;
        private ArmPosition (Rotation2d rotation) {
            this.rotation = rotation;
        }
    }
    
    public void setArmSpeed (double input) {
        
        double armDegrees = getArmRotation().getDegrees();
        
        // TODO: Right arm motor current limiter separate from left arm motor
        boolean armCurrentLimitTripped = armCurrentStopSecondDebouncer.calculate(
            armCurrentStopFirstDebouncer.calculate(
                leftArmMotor.getOutputCurrent() > ARM_CURRENT_LIMIT
            )
        );
        
        if (!armCurrentLimitTripped) {
            if ((input >= 0 && armDegrees < ARM_MAX_ANGLE_DEGREES) || (input < 0 && armDegrees > ARM_MIN_ANGLE_DEGREES)) {
                setArmSpeedOverride(input);
            } else stop();
        } else stop();
        
    }
    
    public void stop () {
        armSpeedFilter.reset(0);
        setArmSpeedOverride(0);
    }
    
    @Override
    public void initSendable (SendableBuilder builder) {
        builder.addDoubleProperty("Arm position", () -> getArmRotation().getDegrees(), null);
        builder.addDoubleProperty("Left arm output current", () -> leftArmMotor.getOutputCurrent(), null);
        builder.addDoubleProperty("Right arm output current", () -> rightArmMotor.getOutputCurrent(), null);
    }
    
    @Override
    public void periodic () {
        VisionManager.getInstance().manageCamStreams(getArmRotation());
    }
    
}