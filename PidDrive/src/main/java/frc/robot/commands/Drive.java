// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import frc.robot.subsystems.ExampleSubsystem;
// import frc.robot.subsystems.PidDrive;

// import javax.swing.text.StyleContext.SmallAttributeSet;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

// /** An example command that uses an example subsystem. */
// public class Drive extends CommandBase {
//   @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
//   private final PidDrive leftSide;
//   private final PidDrive rightSide;
//   private double setpoint;

//   /**
//    * Creates a new ExampleCommand.
//    *
//    * @param subsystem The subsystem used by this command.
//    */
//   public Drive(PidDrive leftSide, PidDrive rightSide, double setpoint) {
//     SmartDashboard.putNumber("Setpoint", setpoint);
//     this.leftSide = leftSide;
//     this.rightSide = rightSide;
//     this.setpoint = setpoint;

//     addRequirements(leftSide);
//     addRequirements(rightSide);


//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {}

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     leftSide.setSetpoint(setpoint);
//     rightSide.setSetpoint(setpoint);

//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
