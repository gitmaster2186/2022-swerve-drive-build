// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PlatformDockPidCommand_X extends PIDCommand {
  private final static AHRS m_navx = new AHRS(SPI.Port.kMXP, (byte) 200); // NavX connected over MXP

  /** Creates a new PlatformDockPidCommand. */
  public PlatformDockPidCommand_X(DrivetrainSubsystem m_drivetrainSubsystem) {
    
    super(
        // The controller that the command will use
        new PIDController(Constants.kP,Constants.kI, Constants.kD),//P,I,D
        // This should return the measurement
        () -> m_navx.getRoll() ,
        // This should return the setpoint (can also be a constant)
        () -> 0,
        // This uses the output
        output -> {
          // this is input to Plant - aka Drive Sub System
            
            // m_drivetrainSubsystem.drive_pid(
            //   ChassisSpeeds.fromFieldRelativeSpeeds(
            //     DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND * output* -1 ,
            //     0,
            //           0,//DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * constant if needed
            //           m_drivetrainSubsystem.getGyroscopeRotation()));

            m_drivetrainSubsystem.drive_pid_x(output);
            //System.out.println("getRoll()");
        // SmartDashboard.putNumber("getRoll",m_navx.getRoll() );
        // //System.out.println( m_navx.getRoll());
        // //System.out.println("pid_output");
        // //System.out.println(pid_output);
        // SmartDashboard.putNumber("pid_output",output);
        // SmartDashboard.updateValues();    
        });
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drivetrainSubsystem);
    // Configure additional PID options by calling `getController` here.
    
  }
 
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
