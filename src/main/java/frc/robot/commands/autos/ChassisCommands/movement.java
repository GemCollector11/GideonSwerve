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


        //246.23264
        private static final Pose2d generatePose(double x, double y, Rotation2d rot){
            return new Pose2d(x, y, rot);
        }

        public static final Trajectory generateTrajectory(Drivetrain drivetrain , double distanceMetersX, double distanceMetersY, Double degrees) {
        double speed = drivetrain.kMaxSpeed / 2;
        double accel = drivetrain.kmaxAccel;

        double distancex = distanceMetersX * 0.70783932047425234471774907096089 * 0.95165588123334602207841644461363;
        double distancey = distanceMetersY * 0.70783932047425234471774907096089 * 0.95165588123334602207841644461363;
        //0.025760165 = correction factor 

        TrajectoryConfig config = new TrajectoryConfig(speed, accel);

        Pose2d startPose = new Pose2d(0,0, drivetrain.getRotation2d());
        drivetrain.setPose(startPose);

        Pose2d endPose = new Pose2d(
            startPose.getX() + distancex,
            startPose.getY() + distancey, 
            Rotation2d.fromDegrees(Math.IEEEremainder(degrees + startPose.getRotation().getDegrees(),360))
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
