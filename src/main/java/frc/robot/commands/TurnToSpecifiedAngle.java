// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TurnToSpecifiedAngle extends PIDCommand {
  private double angle;
  private double currAngle;
  private Swerve s_Swerve;
  /** Creates a new TurnToSpecifiedAngle. */
  public TurnToSpecifiedAngle(Swerve s_Swerve, double currAngle, double angle) {
    super(
        // The controller that the command will use
        new PIDController(.15, 0, 0),
        // This should return the measurement
        () -> s_Swerve.getDoubleYaw(),
        // This should return the setpoint (can also be a constant)
        () -> currAngle + angle,
        // This uses the output
        output -> {
          // Use the output here
          //if(output < .07 && output > 0){
          //  s_Swerve.drive(new Translation2d(0, 0), .08, true, true);
          //}
          //else if(output > -.07 && output < 0){
          //  s_Swerve.drive(new Translation2d(0, 0), -.08, true, true);
          //}
          //else{
            s_Swerve.drive(new Translation2d(0, 0), output, true, true);
          //}
        });
        this.currAngle = currAngle;
        this.angle = angle;
        this.s_Swerve = s_Swerve;
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(s_Swerve.getDoubleYaw() - (currAngle + angle)) < 10;
  }
}
