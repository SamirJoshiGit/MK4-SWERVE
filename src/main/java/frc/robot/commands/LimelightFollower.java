// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Globals;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LimelightFollower extends PIDCommand {
  /** Creates a new LimelightFollower. */
  private boolean finishAtEnd;
  private Limelight m_Limelight;
  public LimelightFollower(Swerve m_Swerve, Limelight m_Limelight, boolean finishAtEnd, boolean runConcurrently) {
    super(
        // The controller that the command will use
        new PIDController(.35, 0, 0),
        // This should return the measurement
        () -> m_Limelight.limelightOffset(),
        // This should return the setpoint (can also be a constant)
        () -> 0,
        // This uses the output
        output -> {
          // Use the output here
          if(!runConcurrently){
            m_Swerve.drive(new Translation2d(0, 0), output, true, true);
          }
          Globals.rotatingOutput = output;
        });
        this.finishAtEnd= finishAtEnd;
        this.m_Limelight =m_Limelight;  
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      return finishAtEnd && Math.abs(m_Limelight.limelightOffset()) < Math.abs(.3);
  }
}
