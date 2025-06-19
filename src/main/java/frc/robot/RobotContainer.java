package frc.robot;


import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.commands.ManualDrive;
import subsystems.Drivetrain;

public class RobotContainer {

    private final Drivetrain drivetrain = new Drivetrain();

    private final CommandJoystick driveStick = new CommandJoystick(0);

    public RobotContainer(){
        drivetrain.setDefaultCommand(new ManualDrive(drivetrain,
        () -> driveStick.getRawAxis(1),
        () -> driveStick.getRawAxis(2),
        () -> driveStick.getRawAxis(3)
        ));
        configureBindings();
    }
    private void configureBindings() {

       //driveStick.button(1).onTrue(Commands.runOnce(() -> drivetrain.resetAll(), drivetrain));
    }
}
