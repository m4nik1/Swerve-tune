// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

public class TeleopSwerve extends Command {
  /** Creates a new TeleopSwerve. */
  // create the limiters for translation, strafe, and rotation

  public TeleopSwerve() {
    // Use addRequirements() here to declare subsystem dependencies.


    // set the limiters to a value
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // get the stick values

    // set SpeedMulti variable

    // get the turbo (rightBumper)

    // make if statement for turbo button

    // apply the deadbands and set limit for speeds

    // get X and Y into a translation2d

    // mutiply speeds by the max speeds constants and call Drive function


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
