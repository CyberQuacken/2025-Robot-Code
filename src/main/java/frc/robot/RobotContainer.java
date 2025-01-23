// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OIConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.commands.SwerveDriveCommands.lastRotInput;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.VisionSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private final SendableChooser<Command> autoChooser;
  
  // The robot's subsystems and commands are defined here...
  private final SwerveDrive m_robotDrive = new SwerveDrive();
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final VisionSubsystem m_vision = new VisionSubsystem();
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser();
    // Configure the trigger bindings
    configureBindings();
    m_robotDrive.setDefaultCommand(
      new RunCommand(
        () -> m_robotDrive.drive(
          -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDeadband),
          -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDeadband),
          -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDeadband+.2),
        OIConstants.fieldRelative, true), 
        m_robotDrive));
      m_vision.setDefaultCommand(
        new RunCommand( 
          () -> m_vision.run(), m_vision
        )
      );
      
    SmartDashboard.putData("Auto Chooser", autoChooser);

    // <messing around with smartDashboard. it may be good to have a few commands or subsystems for this>
    SmartDashboard.putBoolean(" Home", true);
    SmartDashboard.putBoolean(" intake", false);
    SmartDashboard.putBoolean(" level One", false);
    SmartDashboard.putBoolean(" level Two", false);
    SmartDashboard.putBoolean(" level Three", false);
    SmartDashboard.putBoolean(" level Four", false);
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

    // <code form last year we arent sure what it does>
   /*  new RunCommand( // im not actualy sure what this does anymore
            () -> m_robotDrive.setX(),
            m_robotDrive); */
    


    Trigger rbDriverButton = m_driverController.rightBumper();
    final lastRotInput rotInput = new lastRotInput(m_robotDrive);
    //rbDriverButton.whileTrue(rotInput);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new PathPlannerAuto("New Auto");
  }
  public Command getOtherAutoCommand() { 
    // runs preplanned pathplanner
    return autoChooser.getSelected();
  }
}