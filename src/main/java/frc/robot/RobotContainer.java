package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.ManualDrive;
import subsystems.AutonSubsystem;
import subsystems.Drivetrain;
//#region manuel driveCMD
public class RobotContainer {

    private final Drivetrain drivetrain = new Drivetrain();
    private final AutonSubsystem autonSubsystem = new AutonSubsystem(drivetrain);
    private final Joystick joystick = new Joystick(0, 1, 2, 3, 4);

    public RobotContainer(){

            drivetrain.setDefaultCommand(new ManualDrive(drivetrain,
                () -> joystick.getRawAxis(1),
                () -> joystick.getRawAxis(0),
                () -> joystick.getRawAxis(2),
                () -> joystick.getRawAxis(3) + 1,
                () -> joystick.getAsBoolean(1),
                () -> joystick.getAsBoolean(2),
                () -> joystick.getAsBoolean(3),
                () -> joystick.getAsBoolean(4)
            ));
    
        configureBindings();

    }
    private void configureBindings() {

       //driveStick.button(1).onTrue(Commands.runOnce(() -> drivetrain.resetAll(), drivetrain));
    }

    public final void teleopInit(){

    }

    public final void autoINIT(){

        drivetrain.frontRight.resetDriveEncoder();
        drivetrain.frontLeft.resetDriveEncoder();
        drivetrain.backRight.resetDriveEncoder();
        drivetrain.backLeft.resetDriveEncoder();
    }
    //#endregion
    //#region get Functions

    public final Command getAutoCommand(double x, double y, double angle){
        return autonSubsystem.getCommand(x ,y, angle);
    }

    public final Command getDriveCommand(){
        return drivetrain.getDefaultCommand();
    }
}
