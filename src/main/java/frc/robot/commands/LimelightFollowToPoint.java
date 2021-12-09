// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Globals;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LimelightFollowToPoint extends PIDCommand {
  private boolean finishAtEnd;
  private Limelight m_Limelight;
  private double m_stopPoint;
  /** Creates a new LimelightFollowToPoint. */
  public LimelightFollowToPoint(Swerve m_Swerve, Limelight m_Limelight, boolean finishAtEnd, double m_stopPoint, boolean runConcurrently) {
    super(
        // The controller that the command will use
        new PIDController(1, 0, 0),
        // This should return the measurement
        () -> m_Limelight.limelightArea(),
        // This should return the setpoint (can also be a constant)
        () -> m_stopPoint,
        // This uses the output
        output -> {
          if(m_Limelight.limelightArea() == 0){
            if(!runConcurrently){
              m_Swerve.drive(new Translation2d(0, 0), 0, true, true);
            }
            Globals.movingOutput = 0;
          }
          else{
            if(!runConcurrently){
              m_Swerve.drive(new Translation2d(-output, 0), 0, true, true);
            }
            Globals.movingOutput = output;
          }
          
          // Use the output here
        });
        this.finishAtEnd= finishAtEnd;
        this.m_Limelight =m_Limelight;
        this.m_stopPoint = m_stopPoint;  
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finishAtEnd && (m_stopPoint-m_Limelight.limelightArea() <.07);
  }
}
