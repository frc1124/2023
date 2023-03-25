// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import frc.robot.RobotContainer;
// import frc.robot.subsystems.ExampleSubsystem;
// import frc.robot.subsystems.PidDrive;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import edu.wpi.first.wpilibj2.command.CommandScheduler;
// import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

// /** An example command that uses an example subsystem. */
// public class TurnGroup extends ParallelCommandGroup {
//   @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
//   private final PidDrive leftSide;
//   private final PidDrive rightSide;
//   private double angleSetpoint;
//   private double velSetpoint;

//   /**
//    * Creates a new ExampleCommand.
//    *
//    * @param subsystem The subsystem used by this command.
//    */
//   public TurnGroup(RobotContainer rc, double angleSetpoint, double velSetpoint) {
//     leftSide = rc.leftSide;
//     rightSide = rc.rightSide;
//     this.velSetpoint = velSetpoint;
//     this.angleSetpoint = angleSetpoint;


//     // Use addRequirements() here to declare subsystem dependencies.
//     addRequirements(rc.leftSide);
//     addRequirements(rc.rightSide);

//     CommandScheduler.getInstance().schedule(new Turn(leftSide, angleSetpoint, velSetpoint));
//     CommandScheduler.getInstance().schedule(new Turn(rightSide, angleSetpoint, velSetpoint));


//   }
// }
