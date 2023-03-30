package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.*;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
import com.kauailabs.navx.frc.AHRS;

public class DriveSubsystem extends SubsystemBase {
  // The motors on the left side of the drive.
  private final MotorControllerGroup m_leftMotors;

  // The motors on the right side of the drive.
  private final MotorControllerGroup m_rightMotors;

  // The robot's drive
  private final DifferentialDrive m_drive;

  // The left-side drive encoder
  private final Encoder m_leftEncoder;

  // The right-side drive encoder
  private final Encoder m_rightEncoder;

  // The gyro sensor
  private final AHRS navx;

  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;

  // Kinematics class for movement calculation
  private final DifferentialDriveKinematics m_Kinematics;

  // Sides
  private final PidDrive m_leftSide;
  private final PidDrive m_rightSide;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem(RobotContainer rc) {
    m_leftMotors = rc.leftGroup;
    m_rightMotors = rc.rightGroup;

    m_leftEncoder = rc.leftEncoder;
    m_rightEncoder = rc.rightEncoder;

    m_leftSide = rc.leftSide;
    m_rightSide = rc.rightSide;

    m_Kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(Constants.track_width));
    m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);
    navx = rc.navx;

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightMotors.setInverted(true);

    // Sets the distance per pulse for the encoders
    m_leftEncoder.setDistancePerPulse(Math.PI * 6 / 360);
    m_rightEncoder.setDistancePerPulse(Math.PI * 6 / 360);

    resetEncoders();

    m_odometry =
        new DifferentialDriveOdometry(
            navx.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());            
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        navx.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate());
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(
        navx.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance(), pose);
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param leftSpeed the left speed (inches per second)
   * @param rightSpeed the right speed (inches per second)
   */ 

  public void setSpeeds(double leftSpeed, double rightSpeed) {
    // Set the speeds 
    CommandScheduler.getInstance().schedule(new SpeedSetter(m_leftSide, m_rightSide, leftSpeed, rightSpeed));
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement in inches/s
   * @param rot the commanded rotation in radians
   */ 
  public void arcadeDrive(double fwd, double rot) {
    var chassisSpeeds = new ChassisSpeeds(fwd, 0, rot);
    DifferentialDriveWheelSpeeds  wheelSpeeds = m_Kinematics.toWheelSpeeds(chassisSpeeds);
    setSpeeds(wheelSpeeds.leftMetersPerSecond, wheelSpeeds.rightMetersPerSecond);
  }

  public void driveDistance(double desiredX, double rot) {
    Pose2d pose = m_odometry.getPoseMeters();
    Rotation2d rotation2d = pose.getRotation();

    double currentRot = rotation2d.getRadians();
    double currentX = pose.getX();
    double currentY = pose.getY();

  }


  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMotors.setVoltage(leftVolts);
    m_rightMotors.setVoltage(rightVolts);
    m_drive.feed();
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (m_leftEncoder.getDistance() + m_rightEncoder.getDistance()) / 2.0;
  }

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  public Encoder getLeftEncoder() {
    return m_leftEncoder;
  }

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  public Encoder getRightEncoder() {
    return m_rightEncoder;
  }

  /** 
   *@return the odometry object 
  **/
  public DifferentialDriveOdometry getOdometry() {
    return m_odometry;
  }

  public void stop() {
    setSpeeds(0, 0);
  }

  /** 
   *@return the x distance travelled (as shown by odometry)
  **/
  public double getXDistance() {
    return m_odometry.getPoseMeters().getX();
  }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    navx.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return navx.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -navx.getRate();
  }
}