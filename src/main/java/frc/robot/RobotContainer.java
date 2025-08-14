package frc.robot;

import frc.robot.commands.ManualDrive;
import subsystems.Drivetrain;

public class RobotContainer {

    private final Drivetrain drivetrain = new Drivetrain();
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

    }
    private void configureBindings() {

       //driveStick.button(1).onTrue(Commands.runOnce(() -> drivetrain.resetAll(), drivetrain));
    }
}
