package frc.robot.commands.autos.ChassisCommands;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import subsystems.Drivetrain;

public class movement extends SwerveControllerCommand{

    private final Drivetrain drivetrain;

        public static final Trajectory generateTrajectory(Drivetrain drivetrain , double distanceMetersX, double distanceMetersY, Double degrees) {
        double speed = drivetrain.kMaxSpeed / 2;
        double accel = drivetrain.kmaxAccel;
        boolean isDriveReversed = false;

        double distancex = distanceMetersX;
        double distancey = distanceMetersY;

        if(distanceMetersX < 0){
            isDriveReversed = true;
            //distancex *= -1;
        }


        //0.025760165 = correction factor 

        TrajectoryConfig config = new TrajectoryConfig(speed, accel).setReversed(isDriveReversed);

        Pose2d startPose = new Pose2d(0,0, drivetrain.getRotation2d());
        drivetrain.setPose(startPose);

        Pose2d endPose = new Pose2d(
            startPose.getX() + distancex,
            startPose.getY() + distancey, 
            Rotation2d.fromDegrees(degrees + startPose.getRotation().getDegrees())
        );

        List<Pose2d> poseList = List.of(startPose, endPose);
        return TrajectoryGenerator.generateTrajectory(poseList, config);
    }
    public movement(Drivetrain drivetrain, double distanceMetersX , double distanceMetersY, double angle){

        super( generateTrajectory(drivetrain , distanceMetersX, distanceMetersY, angle),
            drivetrain::getPose,
            drivetrain.getKinematics(),
            drivetrain.getDriveController(),
            drivetrain::setAutonModuleStates,
            drivetrain
        );

        this.drivetrain = drivetrain;
    }
}
