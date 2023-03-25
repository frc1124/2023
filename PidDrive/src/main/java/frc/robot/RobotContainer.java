// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.*;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.net.CacheRequest;
import java.util.HashMap;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  public final XboxController j;
  
  // Gyro 
  private final AHRS navx = new AHRS();


  // PID Controllers
  public final PIDController leftVController = new PIDController(
    Constants.VEL_L_P, Constants.VEL_L_I, Constants.VEL_L_D);
  public final PIDController rightVController = new PIDController(
    Constants.VEL_R_P, Constants.VEL_R_I, Constants.VEL_R_D);

  // Drive Motors
  CANSparkMax leftFront = new CANSparkMax(Constants.LEFT_FRONT, MotorType.kBrushless);
  CANSparkMax leftBack = new CANSparkMax(Constants.LEFT_BACK, MotorType.kBrushless);
  MotorControllerGroup leftGroup = new MotorControllerGroup(leftFront, leftBack);

  CANSparkMax rightFront = new CANSparkMax(Constants.RIGHT_FRONT, MotorType.kBrushless);
  CANSparkMax rightBack = new CANSparkMax(Constants.RIGHT_BACK, MotorType.kBrushless);
  MotorControllerGroup rightGroup = new MotorControllerGroup(rightFront, rightBack);

  //encoders
  public Encoder leftEncoder = new Encoder(4, 5, false, CounterBase.EncodingType.k4X);
  public Encoder rightEncoder = new Encoder(6, 7, true, CounterBase.EncodingType.k4X);


  //public final PidDrive leftSide = new PidDrive(leftGroup, `, navx, leftVController, false);
  //public final PidDrive rightSide = new PidDrive(rightGroup, rightEncoder, navx, rightVController, true);

  // Differential Drive
  public DifferentialDrive differentialDrive = new DifferentialDrive(leftGroup, rightGroup);
  public DiffDrive drive = new DiffDrive(differentialDrive);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    





    j = new XboxController(0);
    // Configure the trigger bindings
    configureBindings();
  }

  public final HashMap<String, JoystickButton> logitechMap = new HashMap<String, JoystickButton>();

  public JoystickButton getKey(String key) { 

    logitechMap.put("1", new JoystickButton(j, 1));
    logitechMap.put("2", new JoystickButton(j, 2));
    logitechMap.put("3", new JoystickButton(j, 3));
    logitechMap.put("4", new JoystickButton(j, 4));
    logitechMap.put("left button", new JoystickButton(j, 5));
    logitechMap.put("right button", new JoystickButton(j, 6));
    logitechMap.put("left trigger", new JoystickButton(j, 7));
    logitechMap.put("right trigger", new JoystickButton(j, 8));
    logitechMap.put("9", new JoystickButton(j, 9));
    logitechMap.put("10", new JoystickButton(j, 10));
    logitechMap.put("left joystick", new JoystickButton(j, 11));
    logitechMap.put("right joystick", new JoystickButton(j, 12));

    return logitechMap.get(key);
  }
  
  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Keybinds
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
