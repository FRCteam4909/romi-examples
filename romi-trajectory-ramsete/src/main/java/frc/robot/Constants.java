// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    //The following are based on a mix of the template values and Robot characterization values
    //Static gain
    public static final double ksVolts = 0.929; 
    //Velocity Gain
    public static  final double kvVoltSecondsPerMeter = 6.33;
    //Acceleration gain
    public static final double kaVoltSecondsSquaredPerMeter = 0.0398;

    //P value for ramsete PID controller
    public static final double kPDriveVel = 5.95;
    // The theoretical distance between the axisies of the two wheels on one axel 
    //ie. the distance between the two wheels of the robot.
    public static final double kTrackwidthMeters = 0.142072613; //0.142072613 0.046020801
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 0.8;
    public static final double kMaxAccelerationMetersPerSecondSquared = 0.8;

    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    //b values (> 0) adds to the turning of the robot like a proportional (P) constant
    public static final double kRamseteB = 2;
    //Dampens the b term and the greater the zeta (0 < zeta < 1) the greater the dampening.
    public static final double kRamseteZeta = 0.7;
  }
}
