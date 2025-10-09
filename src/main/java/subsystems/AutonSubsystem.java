package subsystems;

import java.security.PrivateKey;
import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.autos.ChassisCommands.movement;

public class AutonSubsystem extends SubsystemBase{

    private Drivetrain drivetrain;
    //private final movement movement = new movement(drivetrain, 0, 0);

    public AutonSubsystem(Drivetrain drivetrain){
        this.drivetrain = drivetrain;
    }
    
    public final Command getCommand(double x, double y, double angle){
        Command command = new movement(drivetrain, x, y, angle);
        return command;
    }
}
