// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import claw.subsystems.SubsystemCLAW;


public class Swerve extends SubsystemCLAW {

  public Swerve() {}

  public CommandBase exampleMethodCommand() {

    return runOnce(
        () -> {

        });
  }


  public boolean exampleCondition() {

    return false;
  }

  @Override
  public void periodic() {

  }

  @Override
  public void simulationPeriodic() {

  }
}
