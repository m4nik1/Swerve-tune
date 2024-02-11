// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class drivetrain extends SubsystemBase {
  /** Creates a new drivetrain. */
  ElmCitySwerve[] swervemods;

  // Create pigeon gyro
  Pigeon2 gyro;

  // Create the field

  public drivetrain() {
    swervemods=new ElmCitySwerve[] {
      new ElmCitySwerve(0, 18, 19, false, true),
      new ElmCitySwerve(1, 4, 3, false,true),
      new ElmCitySwerve(2, 20, 17, true, true),
      new ElmCitySwerve(3, 2, 1, true, true)
    };

    gyro = new Pigeon2(10);
    zerogyro();
  }

  // set angle function for one module
  public void Setangle(){
    for (ElmCitySwerve  m : swervemods) {
      swervemods[m.moduleNum].angleSet();
    }
  }
  public void setVelocity(){
    for(ElmCitySwerve m : swervemods) {
      swervemods[m.moduleNum].setvelocity();
    }

  }

  // set module states to drive


  // get swerve module positions

  // get robot heading/angle
  public Rotation2d getrobotangle() {
    return gyro.getRotation2d();
  }



  // set gyro to zero 
  public void zerogyro() {
    gyro.setYaw(0);
  }



  public void angleControl() {
    for(ElmCitySwerve m : swervemods){
      m.angleSet();
    }
  }


  @Override

  public void periodic(
  ) {
    // This method will be called once per scheduler run

    // sent robot angle to dashboard
    SmartDashboard.putNumber("Robot angle", getrobotangle().getDegrees());
    // getting the module values
    for( ElmCitySwerve v : swervemods) {
      SmartDashboard.putNumber("angle read " + v.moduleNum, v.angleread());
      SmartDashboard.putNumber("velocity " + v.moduleNum, v.driveVelocity());
      // SmartDashboard.putNumber("Cancoder read" + v.moduleNum, v.getnac());
    }
  }
}

