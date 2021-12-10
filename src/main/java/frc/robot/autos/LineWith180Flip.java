// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import java.util.List;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.LimelightFollower;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LineWith180Flip extends SequentialCommandGroup {
  public LineWith180Flip(Swerve s_Swerve, Limelight m_Limelight){
    TrajectoryConfig config =
        new TrajectoryConfig(
                Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared).setReversed(true)
            .setKinematics(Constants.Swerve.swerveKinematics);


    // An example trajectory to follow.  All units in meters.
    Trajectory firstWaypoint = 
    TrajectoryGenerator.generateTrajectory(
        List.of(new Pose2d(0, 0, new Rotation2d(0)),
            //new Pose2d(0, 0, new Rotation2d(Units.degreesToRadians(-90))) 
            new Pose2d(-Units.feetToMeters(5)-AutoConstants.kOffset, -AutoConstants.kOffsetSide, new Rotation2d(0)),
            new Pose2d(-Units.feetToMeters(5)-AutoConstants.kOffset, .5, new Rotation2d(0))
            //new Pose2d(-Units.feetToMeters(5)-AutoConstants.kOffset, -AutoConstants.kOffsetSide, new Rotation2d(Units.degreesToRadians(-90)))
            ), 
        config);

    var thetaController =
        new ProfiledPIDController(
            Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            firstWaypoint,
            s_Swerve::getPose,
            Constants.Swerve.swerveKinematics,
            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
            thetaController,
            s_Swerve::setModuleStates,
            s_Swerve);


    addCommands(
        new InstantCommand(() -> s_Swerve.resetOdometry(firstWaypoint.getInitialPose())),
        swerveControllerCommand, new InstantCommand(() ->  s_Swerve.drive(new Translation2d(0, 0), 0, true, true)), 
        new LimelightFollower(s_Swerve, m_Limelight, false, false) 
        //
    );
}
}
