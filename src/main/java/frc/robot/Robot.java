// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import subsystems.Drivetrain;

public class Robot extends TimedRobot {
  //private final XboxController controller = new XboxController(0);
  //private final Drivetrain swerve = new Drivetrain();

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  //private final SlewRateLimiter xspeedLimiter = new SlewRateLimiter(3);
  //private final SlewRateLimiter yspeedLimiter = new SlewRateLimiter(3);
  //private final SlewRateLimiter rotLimiter = new SlewRateLimiter(3);
  private final RobotContainer robotContainer;
  //private Command autonomusCommands;
 
  public Robot() {
    robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    //CommandScheduler.getInstance().run();
  }


  @Override
  public void autonomousPeriodic() {
    //CommandScheduler.getInstance().run();
  }

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    CommandScheduler.getInstance().run();
    //driveWithJoystick(true);
  }

  /*private void driveWithJoystick(boolean fieldRelative) {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    final var xSpeed =
        -xspeedLimiter.calculate(MathUtil.applyDeadband(controller.getLeftY(), 0.02))
            * Drivetrain.kMaxSpeed;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    final var ySpeed =
        -yspeedLimiter.calculate(MathUtil.applyDeadband(controller.getLeftX(), 0.02))
            * Drivetrain.kMaxSpeed;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    final var rot =
        -rotLimiter.calculate(MathUtil.applyDeadband(controller.getRightX(), 0.02))
            * Drivetrain.kMaxAngularSpeed;

    swerve.drive(xSpeed, ySpeed, rot, fieldRelative, getPeriod());
  }*/
}
