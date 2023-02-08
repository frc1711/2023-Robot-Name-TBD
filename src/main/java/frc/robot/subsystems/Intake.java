// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.IDMap;

public class Intake extends SubsystemBase {
  
  private static Intake intakeInstance;

  public static Intake getInstance() {
    if (intakeInstance == null) intakeInstance = new Intake(
                                                      new CANSparkMax(IDMap.INTAKE_LEFT_ARM, MotorType.kBrushless),
                                                      new CANSparkMax(IDMap.INTAKE_RIGHT_ARM, MotorType.kBrushless), 
                                                      new CANSparkMax(IDMap.INTAKE_TOP_BAR, MotorType.kBrushless), 
                                                      new CANSparkMax(IDMap.INTAKE_LOWER_BAR, MotorType.kBrushless),
                                                      new DigitalInput(IDMap.INTAKE_LEFT_SWITCH),
                                                      new DigitalInput(IDMap.INTAKE_RIGHT_SWITCH));
    return intakeInstance;
  }

  private CANSparkMax leftArm, rightArm, topIntakeBar, lowerIntakeBar;
  private DigitalInput leftLimitSwitch, rightLimitSwitch;

  public Intake(
              CANSparkMax leftArm,
              CANSparkMax rightArm,
              CANSparkMax topIntakeBar,
              CANSparkMax lowerIntakeBar,
              DigitalInput leftLimitSwitch,
              DigitalInput rightLimitSwitch
              ) {
        this.leftArm = leftArm;
        this.rightArm = rightArm;
        this.topIntakeBar = topIntakeBar;
        this.lowerIntakeBar = lowerIntakeBar;
        this.leftLimitSwitch = leftLimitSwitch;
        this.rightLimitSwitch = rightLimitSwitch;
        }

  @Override
  public void periodic() {  }
}
