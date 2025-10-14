// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package subsystems;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.SwerveModule;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain extends SubsystemBase{
  public final double kMaxSpeed = 4.2; // 4.2 meters per second
  public static final double kmaxAccel = 4.2;
  public static final double kMaxAngularSpeed = 180; // 1/2 rotation per second
//private final DifferentialDriveOdometry odometry;
    private final PIDController yErrorPIDController = new PIDController(0.066, 0, 0);
    private final PIDController xErrorPIDController = new PIDController(0.066, 0, 0);
    private final ProfiledPIDController autonThetaController = new ProfiledPIDController(0.066, 0, 0, new Constraints(Math.PI * 4, kmaxAccel)); // 1.5

  private Translation2d frontLeftLocation = new Translation2d(0.3125, 0.3125);
  private Translation2d frontRightLocation = new Translation2d(-0.3125, 0.3125);
  private Translation2d backLeftLocation = new Translation2d(0.3125, -0.3125);
  private Translation2d backRightLocation = new Translation2d(-0.3125, -0.3125);

  public final SwerveModule frontLeft = new SwerveModule(1, 2, 0,false, false);
  public final SwerveModule frontRight = new SwerveModule(3, 4, 1,false,false);
  public final SwerveModule backLeft = new SwerveModule(5, 6, 2, false, false);
  public final SwerveModule backRight = new SwerveModule(7, 8, 3,false, false);

  public final SwerveModule[] swerveModules = {backLeft, frontLeft, backRight, frontRight};

  public final AHRS gyro = new AHRS(NavXComType.kUSB1);
  private double rotationXVal;
  private double rotationYval;


  private SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(
          frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);

  public void setRotationPoint(int rotpoint, double meterOffSet){
    double yOffset;
    double xOffset;
    if(rotpoint == 1){
      yOffset = 0;
      xOffset = 0.625 + meterOffSet;
    }else if(rotpoint == 3){
      yOffset = 0;
      xOffset = -0.625 - meterOffSet;
    }else if(rotpoint == 4){
      yOffset = 0.625 + meterOffSet;
      xOffset = 0;
    }else if (rotpoint == 2){
      yOffset = -0.625 - meterOffSet;
      xOffset = 0;
    }else{
      yOffset = 0;
      xOffset = 0;
    }

    rotationXVal = xOffset;
    rotationYval = yOffset;

    frontLeftLocation = new Translation2d(0.3125 + xOffset, 0.3125 + yOffset);
    frontRightLocation = new Translation2d(-0.3125 + xOffset, 0.3125 + yOffset);
    backLeftLocation = new Translation2d(0.3125 + xOffset, -0.3125 + yOffset);
    backRightLocation = new Translation2d(-0.3125 + xOffset, -0.3125 + yOffset);

    kinematics =
      new SwerveDriveKinematics(
        frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);
        updateOdometry();

    odometry =
      new SwerveDriveOdometry(
        kinematics,
        gyro.getRotation2d(),
        new SwerveModulePosition[] {
          frontLeft.getPosition(),
          frontRight.getPosition(),
          backLeft.getPosition(),
          backRight.getPosition()
    });
  }

  private SwerveDriveOdometry odometry =
      new SwerveDriveOdometry(
          kinematics,
          gyro.getRotation2d(),
          new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
          });

  public Drivetrain() {
    gyro.reset();
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */


  public void drive(
    double xSpeed, double ySpeed, double rot, boolean fieldRelative, double periodSeconds) {

    SmartDashboard.putNumber("rotVal:", rot * 2);

    var swerveModuleStates =
        kinematics.toSwerveModuleStates(
            ChassisSpeeds.discretize(
                fieldRelative
                    ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        xSpeed, ySpeed, rot * 2, getRotation2d())
                    : new ChassisSpeeds(MathUtil.clamp(xSpeed, -1.0, 1.0), MathUtil.clamp(ySpeed, -1.0, 1.0), MathUtil.clamp(rot, -2.0, 2.0)),
                periodSeconds));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
    frontLeft.setDesiredState(swerveModuleStates[1]);
    frontRight.setDesiredState(swerveModuleStates[3]);
    backLeft.setDesiredState(swerveModuleStates[0]);
    backRight.setDesiredState(swerveModuleStates[2]);
  }

  public double getHeading(){
    return Math.IEEEremainder(gyro.getAngle(), 360);
  }

  public double getAbsoluteHeading(){
    return gyro.getAngle();
  }

  public Rotation2d getRotation2d(){
    return Rotation2d.fromDegrees(getHeading());
  }

  public SwerveDriveKinematics getKinematics(){
    return kinematics;
  }

  public HolonomicDriveController getDriveController(){
    return new HolonomicDriveController(
      xErrorPIDController,
      yErrorPIDController,
      autonThetaController
      );
  }

  public Pose2d getPose(){
    return odometry.getPoseMeters() ;
  }

  public void setAutonModuleStates(SwerveModuleState[] desiredStates) {
    for (int i = 0; i < 4; i++) {
      swerveModules[i].setDesiredState(desiredStates[i]);
    }
  }
  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    odometry.update(
        gyro.getRotation2d(),
        new SwerveModulePosition[] {
          frontLeft.getPosition(),
          frontRight.getPosition(),
          backLeft.getPosition(),
          backRight.getPosition()
        });
  }
  @Override
    public void periodic() {
        SmartDashboard.putNumber("xoffset:", rotationXVal);
        SmartDashboard.putNumber("yoffset:", rotationYval);  
        SmartDashboard.putNumber("robot angle:", getHeading());
        SmartDashboard.putNumber("abs robot angle:", getAbsoluteHeading());
        SmartDashboard.putNumber("drive encoder(DE):", (frontLeft.getDriveEncoder() + frontRight.getDriveEncoder() + backLeft.getDriveEncoder() + backRight.getDriveEncoder())/4);
        SmartDashboard.putNumber("distance travelled meters:", ((frontLeft.getDriveEncoder() + frontRight.getDriveEncoder() + backLeft.getDriveEncoder() + backRight.getDriveEncoder())/4) / 246.23264);
        updateOdometry();
        super.periodic();
    }
    public void setPose(Pose2d pose) {
        odometry.resetPosition(
            getRotation2d(),
            new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
            },
            pose
        );
    }
}
