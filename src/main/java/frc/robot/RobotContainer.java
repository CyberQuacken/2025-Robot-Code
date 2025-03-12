// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OIConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.algaeHarvesterConstants;
import frc.robot.Constants.algeaScrubberConstants;
import frc.robot.Constants.coralFeederConstants;
import frc.robot.Constants.elevatorConstants;
import frc.robot.commands.toggleAlignment;
import frc.robot.commands.toggleLimelightAuto;
import frc.robot.commands.CoralFeederCommands.manuoverCoral;
import frc.robot.commands.CoralFeederCommands.pathPlannerCoral;
import frc.robot.commands.SwerveDriveCommands.resetGyroCommand;
import frc.robot.commands.algaeHarvesterCommands.algaeHarvesterIntakeCommand;
import frc.robot.commands.algaeHarvesterCommands.algaeHarvesterPivotDownCommand;
import frc.robot.commands.algaeHarvesterCommands.algaeHarvesterPivotUpCommand;
import frc.robot.commands.moveElevatorCommands.moveElevatorDownCommand;
import frc.robot.commands.moveElevatorCommands.moveElevatorIntakeCommand;
import frc.robot.commands.moveElevatorCommands.moveElevatorUpCommand;
import frc.robot.commands.moveElevatorCommands.resetEncoderCommand;
import frc.robot.subsystems.CoralFeederSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.AlgaeSubsytems.Harvester.algaeHarvesterIntakeSubsystem;
import frc.robot.subsystems.AlgaeSubsytems.Harvester.algaeHarvesterPivot;
import frc.robot.subsystems.AlgaeSubsytems.Scrubber.AlgaeScrubberPivotSubsytem;
import frc.robot.subsystems.AlgaeSubsytems.Scrubber.AlgaeScrubberSubsystem;
import frc.robot.subsystems.DriveSubsytems.SwerveDriveMananger;

import java.lang.management.OperatingSystemMXBean;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private final SendableChooser<Command> autoChooser;
  


  // The robot's subsystems and commands are defined here...
  private final ElevatorSubsystem m_Elevator = new ElevatorSubsystem(elevatorConstants.leftMotorCanID, elevatorConstants.rightMotorCanID);
  private final CoralFeederSubsystem m_Coral = new CoralFeederSubsystem(coralFeederConstants.motorID);
  private final algaeHarvesterIntakeSubsystem m_algaeIntake = new algaeHarvesterIntakeSubsystem(algaeHarvesterConstants.intakeMotorCANID);
  private final algaeHarvesterPivot m_algaeHarvesterPivot = new algaeHarvesterPivot(algaeHarvesterConstants.pivotMotorCANID);
  
  public final SwerveDriveMananger m_DriveMananger = new SwerveDriveMananger(null);
  //private final algaeHarvesterPivot m_AlgaeHarvesterPivot = new algaeHarvesterPivot(998);//TODO: find actual id, i just put a placeholder
  private final VisionSubsystem m_VisionSubsystem = new VisionSubsystem();
  //private final TestLightCommand testLightCommand = new TestLightCommand(m_VisionSubsystem, m_robotDrive);

  private final toggleLimelightAuto toggleLimelight = new toggleLimelightAuto(m_DriveMananger);


  private final AlgaeScrubberPivotSubsytem m_scrubberPivot = new AlgaeScrubberPivotSubsytem(algeaScrubberConstants.algeaScrubberPivotMotorID);
  private final AlgaeScrubberSubsystem m_scrubber = new AlgaeScrubberSubsystem(algeaScrubberConstants.algeaScrubberMotorID);

  private final toggleAlignment toggleAlignment = new toggleAlignment(m_DriveMananger);
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_scoringController =
      new CommandXboxController(OperatorConstants.kScorerControllerPort);
  private final VisionSubsystem m_vision = new VisionSubsystem();

  //Simple variable names, if yall want them to be more descriptive they can be changed.
  private final moveElevatorDownCommand eDown = new moveElevatorDownCommand(m_Elevator);
  //private final moveElevatorHomeCommand eHome = new moveElevatorHomeCommand(m_Elevator);
  private final moveElevatorIntakeCommand eIntake = new moveElevatorIntakeCommand(m_Elevator);
  private final moveElevatorUpCommand eUp = new moveElevatorUpCommand(m_Elevator);

  private final resetEncoderCommand resetEncoder = new resetEncoderCommand(m_Elevator);
  private final algaeHarvesterPivotUpCommand pivotHarvesterUp = new algaeHarvesterPivotUpCommand(m_algaeHarvesterPivot);
  private final algaeHarvesterPivotDownCommand pivotHarvesterDown = new algaeHarvesterPivotDownCommand(m_algaeHarvesterPivot);
  private final algaeHarvesterIntakeCommand harvesterIntake = new algaeHarvesterIntakeCommand(m_algaeIntake);
  private final resetGyroCommand resetGyro = new resetGyroCommand(m_DriveMananger);
  private final manuoverCoral intakeCoral = new manuoverCoral(m_Coral);
  private final pathPlannerCoral namedAutoCoral = new pathPlannerCoral(m_Coral);
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    SmartDashboard.putNumber("elevator P", elevatorConstants.kP);
    SmartDashboard.putNumber("elevator I", elevatorConstants.kI);
    SmartDashboard.putNumber("elevator D", elevatorConstants.kD);
        autoChooser = AutoBuilder.buildAutoChooser();

    LimelightHelpers.setPipelineIndex("", 1);

    // Configure the trigger bindings
    configureBindings();

    LimelightHelpers.setPipelineIndex("", 1);
    
    /* 
    m_DriveMananger.setDefaultCommand(
      new RunCommand( ()-> m_DriveMananger.ManageSwerveSystem(
        -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDeadband),
        -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDeadband),
        -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDeadband),
        true, false),m_DriveMananger));
      */


    /* 
    m_Elevator.setDefaultCommand(
      new RunCommand( ()-> m_Elevator.moveElevator(-MathUtil.applyDeadband(m_scoringController.getLeftY(), OIConstants.kDeadband)), m_Elevator)
    );
  */
    

  
    m_Elevator.setDefaultCommand(
      new RunCommand( ()-> m_Elevator.moveMotors(), m_Elevator)
    );
    
  
    
    m_scrubberPivot.setDefaultCommand(
      //new RunCommand( ()-> m_scrubberPivot.testMotor(-MathUtil.applyDeadband(m_scoringController.getLeftY(), OIConstants.kDeadband)), m_scrubberPivot)
      new RunCommand(()-> m_scrubberPivot.testMotor(-MathUtil.applyDeadband(m_scoringController.getRightY(), OIConstants.kDeadband)), m_scrubberPivot)
      );
    m_scrubber.setDefaultCommand(
      new RunCommand( ()-> m_scrubber.testMotor(-MathUtil.applyDeadband(m_scoringController.getRightY(), OIConstants.kDeadband)), m_scrubber)
    );
      
            

    m_vision.setDefaultCommand(
      new RunCommand( 
        () -> m_vision.run(), m_vision
      )
    );

    SmartDashboard.putData("Auto Chooser", autoChooser);

    // Named commands

    NamedCommands.registerCommand("eUp", eUp);
    NamedCommands.registerCommand("Score", namedAutoCoral);
    NamedCommands.registerCommand("eDown", eDown);
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


    Trigger aDriverButton = m_driverController.a();
    //aDriverButton.toggleOnTrue(toggleLimelight);

    Trigger bDriverButton = m_driverController.b();
    //bDriverButton.toggleOnTrue(toggleAlignment);
    bDriverButton.onTrue(resetGyro);
    Trigger yDriverButton = m_driverController.y();
    //yDriverButton.whileTrue(testLightCommand);

    Trigger xDriverButton = m_driverController.x();
    //xDriverButton.whileTrue(lastRots);

    //Trigger scorerLeftY = m_scoringController.leftStick();

    Trigger aScorerButton = m_scoringController.a();
    aScorerButton.toggleOnTrue(eIntake);

    Trigger bScorerButton = (m_scoringController.b());
        bScorerButton.whileTrue(intakeCoral);
    
        Trigger xScoreButton = m_scoringController.x();
        /* xScoreButton.whileTrue(new RunCommand(()-> m_Coral.testMotor(-.4), m_Coral));
        xScoreButton.whileFalse(new RunCommand(()-> m_Coral.testMotor(0), m_Coral));
     *////Not on button sheet
        Trigger yScoreButton = m_scoringController.y();
        yScoreButton.whileTrue(resetEncoder);
    
        Trigger downScorerButton = m_scoringController.povDown();
        downScorerButton.toggleOnTrue(eDown); // make while true for hold and mov
        //downScorerButton.toggleOnFalse(null);
    
        Trigger upScorerButton = m_scoringController.povUp();
        upScorerButton.toggleOnTrue(eUp);
        //upScorerButton.toggleOnFalse();
        //upScorerButton.toggleOnTrue(eUp); // make while true for hold and move
        Trigger rightScorerBumper = m_scoringController.rightBumper();
        rightScorerBumper.whileTrue(Commands.parallel(pivotHarvesterDown,harvesterIntake)).whileFalse(Commands.parallel(pivotHarvesterUp));

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