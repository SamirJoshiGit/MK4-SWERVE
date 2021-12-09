// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Globals;
import frc.robot.subsystems.Swerve;

public class moveWithManualInput extends CommandBase {
  private Swerve s_Swerve;
  private double rotate;
  private double xTranslate;
  private double yTranslate; 
  /** Creates a new moveWithManualInput. */
  public moveWithManualInput(Swerve s_Swerve, double rotate, double xTranslate, double yTranslate) {
    this.s_Swerve = s_Swerve;
    this.rotate = rotate;
    this.xTranslate = xTranslate;
    this.yTranslate = yTranslate;
    addRequirements(s_Swerve);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_Swerve.drive(new Translation2d(-Globals.movingOutput, yTranslate), Globals.rotatingOutput, false, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_Swerve.drive(new Translation2d(0, 0), 0, true, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
