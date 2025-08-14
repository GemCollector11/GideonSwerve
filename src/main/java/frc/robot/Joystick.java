package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Joystick{

    public CommandJoystick driveStick;

    public Trigger ROTtrigger1;
    public Trigger ROTtrigger2;
    public Trigger ROTtrigger3;
    public Trigger ROTtrigger4;
    

    public Joystick(int port, int button1, int button2, int button3, int button4){
        driveStick = new CommandJoystick(port);
        ROTtrigger1 = driveStick.button(button1);
        ROTtrigger2 = driveStick.button(button2);
        ROTtrigger3 = driveStick.button(button3);
        ROTtrigger4 = driveStick.button(button4);
    }

    public double getRawAxis(int axis){
        return driveStick.getRawAxis(axis);
    }
    public boolean getAsBoolean(int button){
        boolean isButtonPressed;
        isButtonPressed = ROTtrigger1.getAsBoolean();

        if(button == 1){
            isButtonPressed = ROTtrigger1.getAsBoolean();
        }
        if(button == 2){
            isButtonPressed = ROTtrigger2.getAsBoolean();
        }
        if(button == 3){
            isButtonPressed = ROTtrigger3.getAsBoolean();
        }
        if(button == 4){
            isButtonPressed = ROTtrigger4.getAsBoolean();
        }
        return isButtonPressed;
    };
}
