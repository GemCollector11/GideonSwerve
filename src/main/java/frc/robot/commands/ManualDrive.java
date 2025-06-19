package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import subsystems.Drivetrain;

public class ManualDrive extends Command {
 //Constants constants;
 //   MotorID motorID;
    Drivetrain drivetrain;

    private Supplier<Double> leftXAxis;
    private Supplier<Double> leftYAxis;
    private Supplier<Double> rotationZaxis;
    private double driveLeftAxis;
    private double driveRightAxis;
    private double driveRotation;
    private double XSpeed;
    private double YSpeed;
    private double rotationSpeed;

    public ManualDrive(Drivetrain drivetrain,
     Supplier<Double>leftXAxis, Supplier<Double>leftYAxis,Supplier<Double> rotationZaxis) {
        this.drivetrain = drivetrain;
        this.leftXAxis = leftXAxis;
        this.leftYAxis = leftYAxis;
        this.rotationZaxis = rotationZaxis;
        addRequirements(this.drivetrain);
    }
     @Override
    public void execute() {
        driveLeftAxis = -leftXAxis.get();
        driveRightAxis = -leftYAxis.get();
        driveRotation = -rotationZaxis.get();

        XSpeed = MathUtil.applyDeadband(driveLeftAxis, 0.05);
        YSpeed = MathUtil.applyDeadband(driveRightAxis, 0.05);
        rotationSpeed= MathUtil.applyDeadband(driveRotation, 0.05);
        
        drivetrain.drive(XSpeed, YSpeed, rotationSpeed, false, 0.02);
    }
}
