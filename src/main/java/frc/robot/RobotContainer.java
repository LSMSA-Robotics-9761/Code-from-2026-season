// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.Commands.AutoDrive;
import frc.robot.Commands.Shooter.flywheel;
import frc.robot.subsystems.DriveSubsystem;
// import frc.robot.subsystems.Elevator;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  // private final Elevator m_elevator = new Elevator();

  private boolean fieldCentered = false;

  // The driver's controller
  CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  // The elevator controller's controller
  // CommandXboxController m_elevatorCommandController = new CommandXboxController(OIConstants.kElevatorControllerPort);
  
  //SHOOTER COMMAND CODE
  private final flywheel m_intake = new flywheel();

    
    /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband)),
            m_robotDrive));
    // m_elevator.setDefaultCommand(new RunCommand(
        // () -> {
          // Move arm of elevator with x-axis of right joystick using elevtor controller
  //         m_elevator.moveArm(
  //             MathUtil.applyDeadband(m_elevatorCommandController.getRightX(), OIConstants.kDriveDeadband));
  //       },
  //       m_elevator));
  }

  /**
   * Allows driver to swap between field-centered and robot-centered mode.
   */
  //private boolean getRobotOrientationMode() {}
    // if (m_driverController.getYButton()) {
    //   RunCommand ele_up = new RunCommand(() -> {
    //     m_elevator.setPosition(7.0);
    //     System.out.println("elevator UP");
    //   }, m_elevator);

    //   ele_up.execute();
    // }
    // else if (m_driverController.getAButton()){
    //   RunCommand ele_stop = new RunCommand(() -> {
    //     m_elevator.setPosition(0.0);
    //     System.out.println("elevator STOP");
    //   }, m_elevator);

    //   ele_stop.execute();
    // }
    // else if (m_driverController.getXButton()) {
    //   RunCommand ele_down = new RunCommand(() -> {
    //     m_elevator.setPosition(-3.0);
    //     System.out.println("elevator DOWN");
    //   }, m_elevator);

    //   ele_down.execute();
    // }
  //   if (m_driverController.getRightBumperButton()){
  //     fieldCentered = true;
  //     System.out.println("Set to FIELD Centered");
  //   }

  //   else if (m_driverController.getLeftBumperButton()) {
  //     fieldCentered = false;
  //     System.out.println("Set to ROBOT centered");
  //   }
  //   SmartDashboard.putBoolean("Field Centered", fieldCentered);
  //   return fieldCentered;
  // }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of
   * its subclasses ({@linkedu.wpi.first.wpilibj.Joystick} or
   * {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    m_driverController.a().whileTrue(m_intake.setMotor(Volts.of(9)));
    m_driverController.a().whileFalse(m_intake.setMotor(Volts.of(0)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // // Create config for trajectory
    // TrajectoryConfig config = new TrajectoryConfig(
    // AutoConstants.kMaxSpeedMetersPerSecond,
    // AutoConstants.kMaxAccelerationMetersPerSecondSquared)
    // // Add kinematics to ensure max speed is actually obeyed
    // .setKinematics(DriveConstants.kDriveKinematics);

    // // An example trajectory to follow. All units in meters.
    // Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
    // // Start at the origin facing the +X direction
    // new Pose2d(0, 0, new Rotation2d(0)),
    // // Pass through these two interior waypoints, making an 's' curve path
    // List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
    // // End 3 meters straight ahead of where we started, facing forward
    // new Pose2d(3, 0, new Rotation2d(0)),
    // config);

    // var thetaController = new ProfiledPIDController(
    // AutoConstants.kPThetaController, 0, 0,
    // AutoConstants.kThetaControllerConstraints);
    // thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // SwerveControllerCommand swerveControllerCommand = new
    // SwerveControllerCommand(
    // exampleTrajectory,
    // m_robotDrive::getPose, // Functional interface to feed supplier
    // DriveConstants.kDriveKinematics,

    // // Position controllers
    // new PIDController(AutoConstants.kPXController, 0, 0),
    // new PIDController(AutoConstants.kPYController, 0, 0),
    // thetaController,
    // m_robotDrive::setModuleStates,
    // m_robotDrive);

    // // Reset odometry to the starting pose of the trajectory.
    // m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // // Run path following command, then stop at the end.
    // return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0,
    // false));
    return new AutoDrive(m_robotDrive).withTimeout(1);
  }
}
