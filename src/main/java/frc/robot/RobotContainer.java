// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.swerve.Swerve;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {
    
    private final Swerve swerveSubsystem = new Swerve();
    
    public RobotContainer () {
        
    }
    
    public Command getAutonomousCommand () {
        // This can return null to not run a command
        return null;
    }
    
}
