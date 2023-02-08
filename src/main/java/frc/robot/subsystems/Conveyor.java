// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.ejml.ops.ConvertMatrixData;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.IDMap;

public class Conveyor extends SubsystemBase {
  
  private static Conveyor conveyorInstance;

  public static Conveyor getInstance () {
    if (conveyorInstance == null) conveyorInstance = new Conveyor(new CANSparkMax(IDMap.CONVEYOR, MotorType.kBrushless));
    return conveyorInstance;
  }

  CANSparkMax conveyor;

  public Conveyor(CANSparkMax conveyor) {
    this.conveyor = conveyor;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
