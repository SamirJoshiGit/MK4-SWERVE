// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Swerve;

public class NonTrajectoryCommand extends CommandBase {
  /** Creates a new NonTrajectoryCommand. */
  private Swerve s_Drive;
  private boolean fieldRelative;
  private boolean openLoop;
  private int translationAxis;
  private int strafeAxis;
  private int rotationAxis;

  public NonTrajectoryCommand(Swerve s_Drive, int translationAxis, int strafeAxis, int rotationAxis, boolean fieldRelative, boolean openLoop) {
    this.s_Drive = s_Drive;
    //addRequirements(s_Drive);
    this.translationAxis = translationAxis;
    this.strafeAxis = strafeAxis;
    this.rotationAxis = rotationAxis;
    this.fieldRelative = fieldRelative;
    this.openLoop = openLoop;
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
