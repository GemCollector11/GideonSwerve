package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import subsystems.Drivetrain;

public class ManualDrive extends Command {
 //Constants constants;
 //   MotorID motorID;
    Drivetrain drivetrain;

    private Supplier<Boolean> rotPoint1;
    private Supplier<Boolean> rotPoint2;
    private Supplier<Boolean> rotPoint3;
    private Supplier<Boolean> rotPoint4;
    private Supplier<Double> leftXAxis;
    private Supplier<Double> leftYAxis;
    private Supplier<Double> rotationZaxis;
    private Supplier<Double> sliderVAL;
    private double driveLeftAxis;
    private double driveRightAxis;
    private double driveRotation;
    private int rotationPoint;
    private double rotationSpeed;
    private double XSpeed;
    private double YSpeed;
    private double slider;
    

    public ManualDrive(Drivetrain drivetrain,
     Supplier<Double>leftXAxis, Supplier<Double>leftYAxis,Supplier<Double> rotationZaxis,Supplier<Double> sliderVAL,
      Supplier<Boolean> rotPoint1, Supplier<Boolean> rotPoint2, Supplier<Boolean> rotPoint3, Supplier<Boolean> rotPoint4) {
        this.drivetrain = drivetrain;
        this.leftXAxis = leftXAxis;
        this.leftYAxis = leftYAxis;
        this.rotationZaxis = rotationZaxis;
        this.sliderVAL = sliderVAL;
        this.rotPoint1 = rotPoint1;
        this.rotPoint2 = rotPoint2;
        this.rotPoint3 = rotPoint3;
        this.rotPoint4 = rotPoint4;
        addRequirements(this.drivetrain);
    }
     @Override
    public void execute() {
        driveLeftAxis = -leftXAxis.get();
        driveRightAxis = leftYAxis.get();
        driveRotation = -rotationZaxis.get(); 
        slider = -sliderVAL.get(); 

        XSpeed = MathUtil.clamp(MathUtil.applyDeadband(driveLeftAxis,0.15), -1.0, 1.0);
        YSpeed = MathUtil.clamp(MathUtil.applyDeadband(driveRightAxis,0.15), -1.0, 1.0);
        rotationSpeed= MathUtil.clamp(MathUtil.applyDeadband(driveRotation,0.45), -1.0, 1.0);

        if(rotPoint1.get()){
            rotationPoint = 1;
        }else if (rotPoint2.get()){
            rotationPoint = 3;
        }else if(rotPoint3.get()){
            rotationPoint = 2;
        }else if(rotPoint4.get()){
            rotationPoint = 4;
        }else{
            rotationPoint = 0;
        }
        drivetrain.setRotationPoint(rotationPoint, -slider / 2);
        drivetrain.drive(XSpeed, YSpeed, rotationSpeed, true, 0.02);
    }
}
