// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.commands.TestLightCommand;
import frc.robot.commands.toggleLimelightAuto;

import frc.robot.subsystems.SwerveDriveMananger;
import frc.robot.subsystems.VisionSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardComponent;
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
  //public final SwerveDrive m_robotDrive = new SwerveDrive();
  public final SwerveDriveMananger m_DriveMananger = new SwerveDriveMananger(null);
  
  private final VisionSubsystem m_VisionSubsystem = new VisionSubsystem();
  //private final TestLightCommand testLightCommand = new TestLightCommand(m_VisionSubsystem, m_robotDrive);

  private final toggleLimelightAuto toggleLimelight = new toggleLimelightAuto(m_DriveMananger);
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final VisionSubsystem m_vision = new VisionSubsystem();
  //private final lastRotCommand lastRots = new lastRotCommand(m_robotDrive);
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putNumber("limelight_kp_y", DriveConstants.limelight_auto_kp_horizontal);
    SmartDashboard.putNumber("limelight_ki_y", DriveConstants.limelight_auto_ki_horizontal);
    SmartDashboard.putNumber("limelight_kd_y", DriveConstants.limelight_auto_kd_horizontal);

    SmartDashboard.putNumber("limelight_kp_x", DriveConstants.limelight_auto_kp_vertical);
    SmartDashboard.putNumber("limelight_ki_x", DriveConstants.limelight_auto_ki_vertical);
    SmartDashboard.putNumber("limelight_kd_x", DriveConstants.limelight_auto_kd_vertical);
    SmartDashboard.putNumber("inverse X", 1);

    SmartDashboard.putNumber("limelight_kp_z", DriveConstants.limelight_auto_kp_rotation);

    SmartDashboard.putNumber("limelight_horizontalOffset", 0);
    SmartDashboard.putNumber("limelight_verticalOffset", 3);

    LimelightHelpers.setPipelineIndex("", 1);

    // Configure the trigger bindings
    configureBindings();

    LimelightHelpers.setPipelineIndex("", 1);
    LimelightHelpers.setPriorityTagID("",10);
    
    m_DriveMananger.setDefaultCommand(
      new RunCommand( ()-> m_DriveMananger.ManangSwerveSystem(
        -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDeadband),
        -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDeadband),
        -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDeadband),
        true, false),m_DriveMananger));
      

      
      // /* 
      //m_robotDrive.setDefaultCommand(new RunCommand(()-> m_robotDrive.moveRot(1, false), m_robotDrive));
      // */
            

    m_vision.setDefaultCommand(
      new RunCommand( 
        () -> m_vision.run(), m_vision
      )
    );


    SmartDashboard.putData("Auto Chooser", autoChooser);
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

    Trigger aDriverButton = m_driverController.a();
    aDriverButton.toggleOnTrue(toggleLimelight);

    Trigger bDriverButton = m_driverController.b();
    
    Trigger yDriverButton = m_driverController.y();
    //yDriverButton.whileTrue(testLightCommand);

    Trigger xDriverButton = m_driverController.x();
    //xDriverButton.whileTrue(lastRots);
    SmartDashboard.putData("Auto Chooser", autoChooser);
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