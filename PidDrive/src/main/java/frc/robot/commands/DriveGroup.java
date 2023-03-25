// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.PidDrive;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

/** An example command that uses an example subsystem. */
public class DriveGroup extends ParallelCommandGroup {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final PidDrive leftSide;
  private final PidDrive rightSide;
  private double setpoint;


  public DriveGroup(RobotContainer rc, double setpoint) {
    leftSide = rc.leftSide;
    rightSide = rc.rightSide;
    this.setpoint = setpoint;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(rc.leftSide);
    addRequirements(rc.rightSide);

    CommandScheduler.getInstance().schedule(new Drive(leftSide, setpoint));
    CommandScheduler.getInstance().schedule(new Drive(rightSide, setpoint));


  }
}
