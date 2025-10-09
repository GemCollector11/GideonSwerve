package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command DriveCommand;
  private Command autonomousCommand;
  //private int repeatCount = 0;

  private final RobotContainer robotContainer;
  
  public Robot() {
    robotContainer = new RobotContainer();
  }
  

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    robotContainer.autoINIT();
    autonomousCommand = robotContainer.getAutoCommand(1 ,0 ,0);
    if(autonomousCommand != null){
      autonomousCommand.schedule();
    }
    SmartDashboard.putBoolean("is finished?", isAutonomous());
  }


  @Override
  public void autonomousPeriodic() {
    /*if(autonomousCommand != null){
      if(repeatCount < 1){
        if(autonomousCommand.isFinished()){
          autonomousCommand = robotContainer.getAutoCommand(0,1, 180);
          autonomousCommand.schedule();
          repeatCount += 1;
        }
      }
    }*/
  }

  @Override
  public void autonomousExit() {

  }

  @Override
  public void teleopInit() {
    robotContainer.teleopInit();
    if(autonomousCommand != null){
      autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {    
    
    DriveCommand = robotContainer.getDriveCommand();
    if(DriveCommand != null){
      DriveCommand.schedule();
    }
  }

}