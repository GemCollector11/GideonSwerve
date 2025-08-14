package subsystems;




import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.autonomusCommands;



public class AutonomusSubsystem extends SubsystemBase{
    private final Drivetrain drivetrain = new Drivetrain();
    private final autonomusCommands aCommands = new autonomusCommands(drivetrain.frontLeft, drivetrain.frontRight, drivetrain.backLeft, drivetrain.backRight);

    public AutonomusSubsystem (){

    }

}
