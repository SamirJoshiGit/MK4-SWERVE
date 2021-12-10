// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
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
import frc.robot.commands.LimelightFollowToPoint;
import frc.robot.commands.LimelightFollower;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TrenchAuto extends SequentialCommandGroup {
  public TrenchAuto(Swerve s_Swerve, Limelight m_Limelight){
    TrajectoryConfig config =
        new TrajectoryConfig(
                1.0,//max spped
                1.0)//max aceleration squared
            .setKinematics(Constants.Swerve.swerveKinematics);

    // An example trajectory to follow.  All units in meters.

    Trajectory firstWaypoint = 
    TrajectoryGenerator.generateTrajectory(
        List.of(new Pose2d(0, 0, new Rotation2d(0)),
            //new Pose2d(0, 0, new Rotation2d(Units.degreesToRadians(-90))) 
            new Pose2d(-Units.feetToMeters(1)-AutoConstants.kOffset, 0, new Rotation2d(0))
            //new Pose2d(-Units.feetToMeters(5)-AutoConstants.kOffset, -AutoConstants.kOffsetSide, new Rotation2d(Units.degreesToRadians(-90)))
            ), 
        config);

    Trajectory secondWaypoint = 
    TrajectoryGenerator.generateTrajectory(
        List.of(new Pose2d(0, 0, new Rotation2d(0)),
            //new Pose2d(0, 0, new Rotation2d(Units.degreesToRadians(-90))) 
            new Pose2d(-.01, -1.3, new Rotation2d(0)),
            new Pose2d(1.7, -1.3, new Rotation2d(0))
            //new Pose2d(-Units.feetToMeters(5)-AutoConstants.kOffset, -AutoConstants.kOffsetSide, new Rotation2d(Units.degreesToRadians(-90)))
            ), 
        config);

    Trajectory trenchRun =
        TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(-2, -1.5, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(-3, -1.45), new Translation2d(-4, -1.5)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(-5, -1.5, new Rotation2d(0)),
                config);

    Trajectory shootingposition =
        TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(-5, -1.5, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(-4, -1), new Translation2d(-2, -0.5)),
                // End 3 meters straight ahead of where we started, facing forward                      
                new Pose2d(0, 0, new Rotation2d(0)),
                config);

    var thetaController =
        new ProfiledPIDController(
            Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand1 =
        new SwerveControllerCommand(
            firstWaypoint,
            s_Swerve::getPose,
            Constants.Swerve.swerveKinematics,
            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
            thetaController,
            s_Swerve::setModuleStates,
            s_Swerve);

    SwerveControllerCommand swerveControllerCommand2 =
        new SwerveControllerCommand(
            secondWaypoint,
            s_Swerve::getPose,
            Constants.Swerve.swerveKinematics,
            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
            thetaController,
            s_Swerve::setModuleStates,
            s_Swerve);

    SwerveControllerCommand swerveControllerCommand3 =
        new SwerveControllerCommand(
            shootingposition,
            s_Swerve::getPose,
            Constants.Swerve.swerveKinematics,
            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
            thetaController,
            s_Swerve::setModuleStates,
            s_Swerve);

    addCommands(
        new InstantCommand(() -> s_Swerve.resetOdometry(firstWaypoint.getInitialPose())),
        //new LimelightFollower(s_Swerve, m_Limelight, true, false)
        swerveControllerCommand1,
        new LimelightFollowToPoint(s_Swerve, m_Limelight, true, .8, false),
        new InstantCommand(()->s_Swerve.resetOdometry(secondWaypoint.getInitialPose())),
        swerveControllerCommand2
    );
}
}