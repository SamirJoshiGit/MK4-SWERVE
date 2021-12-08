package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.TurnToSpecifiedAngle;
import frc.robot.commands.Wait;
import frc.robot.subsystems.Swerve;

import java.util.List;

import edu.wpi.first.wpilibj.Ultrasonic.Unit;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.EllipticalRegionConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.RectangularRegionConstraint;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class exampleAuto extends SequentialCommandGroup {
    public exampleAuto(Swerve s_Swerve){
        TrajectoryConfig config =
            new TrajectoryConfig(
                    Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                    Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                    .setReversed(true)
                .setKinematics(Constants.Swerve.swerveKinematics);

        // An example trajectory to follow.  All units in meters.
        Trajectory exampleTrajectory =
            TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(-.2, .01), new Translation2d(-.8, -.01)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(-1, 0, new Rotation2d(0)),
                config);
        Trajectory waypointlist = 
            TrajectoryGenerator.generateTrajectory(
                List.of(new Pose2d(0, 0, new Rotation2d(0)),
                    //new Pose2d(0, 0, new Rotation2d(Units.degreesToRadians(-90))) 
                    new Pose2d(-Units.feetToMeters(5)-AutoConstants.kOffset, -AutoConstants.kOffsetSide, new Rotation2d(0))
                    //new Pose2d(-Units.feetToMeters(5)-AutoConstants.kOffset, -AutoConstants.kOffsetSide, new Rotation2d(Units.degreesToRadians(-90)))
                    ), 
                config);

        Trajectory turnrightTrajectory = 
            TrajectoryGenerator.generateTrajectory(
                List.of(
                new Pose2d(0, 0, new Rotation2d(Units.degreesToRadians(0))), 
                new Pose2d(-Units.feetToMeters(1.5), 0, new Rotation2d(0)),
                new Pose2d(-Units.feetToMeters(0.2), Units.feetToMeters(5.1)+AutoConstants.kOffset, new Rotation2d(0))), 
                config);

        var thetaController =
            new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand =
            new SwerveControllerCommand(
                waypointlist,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);
        SwerveControllerCommand swerveControllerLeft =
            new SwerveControllerCommand(
                turnrightTrajectory,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);    


        addCommands(
            new InstantCommand(() -> s_Swerve.resetOdometry(waypointlist.getInitialPose())),
            swerveControllerCommand, 
            new TurnToSpecifiedAngle(s_Swerve, s_Swerve.getDoubleYaw() , 93.5),
            new InstantCommand(() ->  s_Swerve.drive(new Translation2d(0, 0), 0, true, true)),
            new Wait(3), 
            new InstantCommand(() -> s_Swerve.resetOdometry(turnrightTrajectory.getInitialPose())),
            swerveControllerLeft,
            new TurnToSpecifiedAngle(s_Swerve, s_Swerve.getDoubleYaw() , 0)
        );
    }
}