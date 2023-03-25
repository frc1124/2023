// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import edu.wpi.first.wpilibj2.command.CommandBase;
// import edu.wpi.first.wpilibj2.command.PIDSubsystem;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;

// import com.revrobotics.CANSparkMax;
// import com.kauailabs.navx.frc.AHRS;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj.AnalogAccelerometer;
// import edu.wpi.first.wpilibj.Encoder;
// import edu.wpi.first.wpilibj.motorcontrol.MotorController;
// import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;


// public class PidDrive extends PIDSubsystem {
//   /** Creates a new ExampleSubsystem. */
//   private Encoder encoder;
//   private MotorControllerGroup motors;
//   private AHRS navx = new AHRS(); 
//   private PIDController controller;
//   private boolean isLeft;
//   private static PIDController turnController = new PIDController(Constants.TURN_P, Constants.TURN_I, Constants.TURN_D);

//   public PidDrive(MotorControllerGroup motors, Encoder encoder, AHRS navx, PIDController controller, boolean isLeft) {
//     super(controller);
//     this.motors = motors;
//     this.encoder = encoder;
//     this.navx = navx;
//     this.controller = controller;
//     this.isLeft = isLeft;
//     this.encoder.setDistancePerPulse(Math.PI*6/360); // Change (inches per pulse)

//     if(isLeft) {
//       motors.setInverted(true);
//     }
//   }

  
//   public double getMeasurement() {
//     return encoder.getRate(); // inches per second
//   }

//   public void useOutput(double output, double setpoint) {
//     double out = controller.calculate(output, setpoint);
//     double angleError = controller.calculate(getMeasurementAngle(), 0);

//     out = MathUtil.clamp(out, -8, 8);
//     motors.setVoltage(out);
//   }

//   public double getMeasurementAngle() {
//     return navx.getRate(); // Degrees per second
//   }

//   public void turn(double angleSetpoint, double setpoint) {
//     double out = controller.calculate(getMeasurement(), setpoint);
//     double angleError = controller.calculate(getMeasurementAngle(), angleSetpoint); // want max turn speed to be 3 degrees per second

//     if (isLeft) {
//       out += angleError;
//     } else {
//       out -= angleError;
//     }    
//     out = MathUtil.clamp(out, -8, 8);

//     motors.setVoltage(out);
//   }







//   /**
//    * Example command factory method.
//    *
//    * @return a command
//    */
//   public CommandBase exampleMethodCommand() {
//     // Inline construction of command goes here.
//     // Subsystem::RunOnce implicitly requires `this` subsystem.
//     return runOnce(
//         () -> {
//           /* one-time action goes here */
//         });
//   }

//   /**
//    * An example method querying a boolean state of the subsystem (for example, a digital sensor).
//    *
//    * @return value of some boolean subsystem state, such as a digital sensor.
//    */
//   public boolean exampleCondition() {
//     // Query some boolean state, such as a digital sensor.
//     return false;
//   }

//   @Override
//   public void periodic() {
//     // This method will be called once per scheduler run
//   }

//   @Override
//   public void simulationPeriodic() {
//     // This method will be called once per scheduler run during simulation
//   }
// }
