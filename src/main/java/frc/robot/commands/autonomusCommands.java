package frc.robot.commands;

import com.studica.frc.AHRS;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SwerveModule;
import frc.robot.commands.autos.auto1;
import subsystems.AutonomusSubsystem;
import subsystems.Drivetrain;

public class autonomusCommands extends Command{

    public final double gearRatio = 0;
    public final double ticksPerCentimeter = 16.91853;
    private Command autoCommand;
    private double driveAngle;
    private double chassisAngle;
    private AutonomusSubsystem autonomusSubsystem = new AutonomusSubsystem();
    private Drivetrain drivetrain = new Drivetrain();
    private final AHRS  gyro = drivetrain.gyro;

    private SwerveModule FrontLeftModule;
    private SwerveModule FrontRightModule;
    private SwerveModule BackLeftModule;
    private SwerveModule BackRightModule;

    public autonomusCommands(SwerveModule frontLeftModule, SwerveModule frontRightModule,SwerveModule backLeftModule, SwerveModule backRightModule){

       FrontLeftModule = frontLeftModule;
       FrontRightModule = frontRightModule;
       BackLeftModule = backLeftModule;
       BackRightModule = backRightModule;

       driveAngle = frontLeftModule.getAngle();
       drivetrain.updateOdometry();
       chassisAngle = gyro.getAngle();
       addRequirements(drivetrain);
    }

    public void forward(double meters, double velocity){
        while (FrontLeftModule.getDriveEncoder() < (meters - 0.02) * (ticksPerCentimeter * 100)) {
            drivetrain.drive(velocity, 0, 0, false, 0.02);
        }
        drivetrain.drive(0, 0, 0, false, 0.02);
    }

    @Override
    public void execute(){
        //autoCommand = auto1.run();
    }
}