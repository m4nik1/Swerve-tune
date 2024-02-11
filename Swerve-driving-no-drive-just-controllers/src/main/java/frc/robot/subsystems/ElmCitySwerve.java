// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.math.OnboardModuleState;

public class ElmCitySwerve extends SubsystemBase {
  /** Creates a new ElmCitySwerve. */
  CANSparkMax drive;
  CANSparkMax angleMotor;

  RelativeEncoder driveEncoder;
  RelativeEncoder angleEncoder;

  // add drive Controller
  SparkPIDController driveController;
  SparkPIDController angleController; 

  CANcoder nac;
  CANcoderConfiguration nacConfig;
  Rotation2d lastAngle;
  public int moduleNum;

  public ElmCitySwerve(int moduleNum, int driveNum, int angleNum, boolean driveInvert, boolean angleInvert) {
    this.moduleNum = moduleNum;
    drive = new CANSparkMax(driveNum, MotorType.kBrushless);
    drive.restoreFactoryDefaults();
    driveEncoder=drive.getEncoder();
    drive.setInverted(driveInvert);
    // add drive position and velocity conversions
    driveEncoder.setPositionConversionFactor(Constants.drivePosConversion);
    driveEncoder.setVelocityConversionFactor(Constants.driveVelConversion);
    // set the drive PID's
    driveController=drive.getPIDController();
    driveController.setP( .7);
    driveController.setI(0.001);
    driveController.setD(1.2);
    driveController.setFF(0);

    angleMotor = new CANSparkMax(angleNum, MotorType.kBrushless );
    angleMotor.restoreFactoryDefaults();
    angleMotor.setInverted(angleInvert);
    angleMotor.setIdleMode(IdleMode.kBrake);
    driveEncoder = drive.getEncoder();
    angleEncoder = angleMotor.getEncoder(); 
    angleEncoder.setPositionConversionFactor(Constants.anglePosConversion);
    resetAbsolute();
    
    // setting controller PID's
    angleController= angleMotor.getPIDController();
    angleController.setP(.030);

    // nac configuration
    // nacConfig = new CANcoderConfiguration();
    // nacConfig.absoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;

    lastAngle = getstate().angle;
  }

  // set desired state of angle and drive motors
  public void setdesiredstate(SwerveModuleState desiredState){
    desiredState = OnboardModuleState.optimize(desiredState,getstate().angle);

    setSpeed(desiredState);
    setAngle(desiredState);
  }

  public void resetAbsolute(){
    // double absolutePos = (CANCoder funtion) - angleOffsetArg;
    angleEncoder.setPosition(0);
  }

  // set angle function
  public void setAngle(SwerveModuleState desiredstate){
    Rotation2d angle;
    double absolutespeed = Math.abs(desiredstate.speedMetersPerSecond);
    if(absolutespeed <= (Constants.maxAngularSpd*.01)){
      angle = lastAngle;

    }
    else{
      angle = desiredstate.angle;
    }
    angleController.setReference(angle.getDegrees(),ControlType.kPosition);
  }
  // set speed function
  public void setSpeed(SwerveModuleState desiredstate){
    double Speedoutput = desiredstate.speedMetersPerSecond/Constants.maxSpeed;
    drive.set(Speedoutput);
  }

  // stop function for modules

  public void angleSet() {
    angleController.setReference(180, ControlType.kPosition);
  }

  public double angleread() {
    return angleEncoder.getPosition();
  }

  // get CANCoder position multiplied by 360.0

  // get drive position
  public double getdrivepostion(){
    return driveEncoder.getPosition();
  }

  // get drive velocity
  public double driveVelocity(){
     return driveEncoder.getVelocity();
  }
  public void setvelocity(){
    driveController.setReference(.3, ControlType.kVelocity);
    
  }
  

  // get state
  public SwerveModuleState getstate() {
    return new SwerveModuleState(driveEncoder.getVelocity(), Rotation2d.fromDegrees(angleEncoder.getPosition()));
  }

  

  // https://github.com/Frc5572/FRC2022/blob/main/src/main/java/frc/robot/subsystems/Swerve.java

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber(" swerve angle", angleread() );
  }
}
