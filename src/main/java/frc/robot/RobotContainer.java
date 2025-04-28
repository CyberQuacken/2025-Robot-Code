// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
//test
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.algaeHarvesterConstants;
import frc.robot.Constants.algaeScrubberConstants;
import frc.robot.Constants.autoWayPointConstants;
import frc.robot.Constants.coralFeederConstants;
import frc.robot.Constants.elevatorConstants;
import frc.robot.commands.SwerveDriveCommands.toggleAlignment;
import frc.robot.commands.toggleLimelightAuto;
import frc.robot.commands.AlgaeScrubberCommands.moveScrubberIn;
import frc.robot.commands.AlgaeScrubberCommands.moveScrubberOut;
import frc.robot.commands.AlgaeScrubberCommands.nullScrubberCommand;
import frc.robot.commands.AlgaeScrubberCommands.overrideScrubber;
import frc.robot.commands.AlgaeScrubberCommands.overrideScrubberCommand;
import frc.robot.commands.AlgaeScrubberCommands.overrideScrubberPivotCommand;
import frc.robot.commands.AlgaeScrubberCommands.ScrubAlgae;
import frc.robot.commands.AlgaeScrubberCommands.manualScrub;
import frc.robot.commands.CoralFeederCommands.autoCoralIntakeCommand;
import frc.robot.commands.CoralFeederCommands.automateIntakeCoral;
import frc.robot.commands.CoralFeederCommands.maneuverCoral;
import frc.robot.commands.CoralFeederCommands.pathPlannerCoral;
import frc.robot.commands.SwerveDriveCommands.DriveFieldCommand;
import frc.robot.commands.SwerveDriveCommands.DriveRobotCommand;
import frc.robot.commands.SwerveDriveCommands.autoAlignOnCenterCommand;
import frc.robot.commands.SwerveDriveCommands.autoAlignOnLeftCommand;
import frc.robot.commands.SwerveDriveCommands.autoAlignOnRightCommand;
import frc.robot.commands.SwerveDriveCommands.normalSpeedCommand;
import frc.robot.commands.SwerveDriveCommands.reduceSpeedCommand;
import frc.robot.commands.SwerveDriveCommands.resetGyroCommand;
import frc.robot.commands.SwerveDriveCommands.setLimelightAutoOff;
import frc.robot.commands.SwerveDriveCommands.toggleDriveModeCommand;
import frc.robot.commands.algaeHarvesterCommands.algaeHarvesterIntakeCommand;
import frc.robot.commands.algaeHarvesterCommands.algaeHarvesterOuttakeCommand;
import frc.robot.commands.algaeHarvesterCommands.algaeHarvesterPivotDownCommand;
import frc.robot.commands.algaeHarvesterCommands.algaeHarvesterPivotUpCommand;
import frc.robot.commands.moveElevatorCommands.autoL4Command;
import frc.robot.commands.moveElevatorCommands.moveElevator;
import frc.robot.commands.moveElevatorCommands.moveElevatorDownCommand;

import frc.robot.commands.moveElevatorCommands.moveElevatorIntakeCommand;
import frc.robot.commands.moveElevatorCommands.moveElevatorUpCommand;
import frc.robot.commands.moveElevatorCommands.nullElevatorCommand;
import frc.robot.commands.moveElevatorCommands.overrideElevatorCommand;
import frc.robot.commands.moveElevatorCommands.resetEncoderCommand;
import frc.robot.commands.moveElevatorCommands.runElevator;
import frc.robot.subsystems.CoralFeederSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.lightsSubsystems;
import frc.robot.subsystems.AlgaeSubsytems.Harvester.algaeHarvesterIntakeSubsystem;
import frc.robot.subsystems.AlgaeSubsytems.Harvester.algaeHarvesterPivot;
import frc.robot.subsystems.AlgaeSubsytems.Scrubber.AlgaeScrubberPivotSubsytem;
import frc.robot.subsystems.AlgaeSubsytems.Scrubber.AlgaeScrubberSubsystem;
import frc.robot.subsystems.DriveSubsytems.SwerveDrive;
import frc.robot.subsystems.DriveSubsytems.SwerveDriveManager;

import java.lang.management.OperatingSystemMXBean;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private final SendableChooser<Command> autoChooser;
  


  // The robot's subsystems and commands are defined here...


  private final lightsSubsystems m_LED = new lightsSubsystems(0,120);
  
  public final SwerveDriveManager m_DriveManager = new SwerveDriveManager(null);
  private final VisionSubsystem m_VisionSubsystem = new VisionSubsystem();
  //private final TestLightCommand testLightCommand = new TestLightCommand(m_VisionSubsystem, m_robotDrive);

  private final toggleLimelightAuto toggleLimelight = new toggleLimelightAuto(m_DriveManager);

  private final toggleAlignment toggleAlignment = new toggleAlignment(m_DriveManager);
  private final setLimelightAutoOff limelightOff = new setLimelightAutoOff(m_DriveManager);

  private final autoAlignOnLeftCommand alignLeft = new autoAlignOnLeftCommand(m_DriveManager);
  private final autoAlignOnRightCommand alignRight = new autoAlignOnRightCommand(m_DriveManager);
  private final autoAlignOnCenterCommand alignCenter = new autoAlignOnCenterCommand(m_DriveManager);

  private final reduceSpeedCommand reduceSpeed = new reduceSpeedCommand();
  private final normalSpeedCommand normalSpeed = new normalSpeedCommand();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_scoringController =
      new CommandXboxController(OperatorConstants.kScorerControllerPort);
  private final CommandGenericHID buttonPannel = 
      new CommandGenericHID(2);
  

  
  //private final AlgaeScrubberPivotSubsytem m_scrubberPivot = new AlgaeScrubberPivotSubsytem(algaeScrubberConstants.algeaScrubberPivotMotorID,m_scoringController);
  //private final AlgaeScrubberSubsystem m_scrubber = new AlgaeScrubberSubsystem(algaeScrubberConstants.algeaScrubberMotorID,m_scoringController);
  private final ElevatorSubsystem m_Elevator = new ElevatorSubsystem(elevatorConstants.leftMotorCanID, elevatorConstants.rightMotorCanID, m_scoringController);
  private final CoralFeederSubsystem m_Coral = new CoralFeederSubsystem(coralFeederConstants.motorID,coralFeederConstants.sensorPort);

  private final moveElevatorDownCommand eDown = new moveElevatorDownCommand(m_Elevator);
  private final moveElevatorIntakeCommand eIntake = new moveElevatorIntakeCommand(m_Elevator);
  private final moveElevatorUpCommand eUp = new moveElevatorUpCommand(m_Elevator);

  private final moveElevator eLevelFour = new moveElevator(m_Elevator, 4);
  private final moveElevator eLevelThree = new moveElevator(m_Elevator, 3);
  private final moveElevator eLevelTwo = new moveElevator(m_Elevator, 2);
  private final moveElevator eLevelOne = new moveElevator(m_Elevator, 1);
  private final overrideElevatorCommand EOverride = new overrideElevatorCommand(m_Elevator);
  private final nullElevatorCommand nullE = new nullElevatorCommand(m_Elevator);
  private final runElevator runElevatorCommand = new runElevator(m_Elevator);

 private final resetEncoderCommand resetEncoder = new resetEncoderCommand(m_Elevator);
  private final resetGyroCommand resetGyro = new resetGyroCommand(m_DriveManager);

  private final maneuverCoral intakeCoral = new maneuverCoral(m_Coral);
  private final pathPlannerCoral namedAutoCoral = new pathPlannerCoral(m_Coral);
  private final automateIntakeCoral intakeCoral2 = new automateIntakeCoral(m_Coral);
  private final autoCoralIntakeCommand autoCoralIntake = new autoCoralIntakeCommand(m_Coral);

  
  private final toggleDriveModeCommand toggleMode = new toggleDriveModeCommand();//This is a real duct tape fix that has a probability of not working.
  private final DriveFieldCommand fieldCentricC = new DriveFieldCommand();
  private final DriveRobotCommand robotCentricC = new DriveRobotCommand();
  private final autoL4Command autoL4 = new autoL4Command(m_Elevator);

  

  Command intakeBlueLeftCommand = AutoBuilder.pathfindToPose(autoWayPointConstants.intakeBlueLeftPosition,autoWayPointConstants.constraints,0.0); // Goal end velocity in meters/sec);
  Command intakeBlueRightCommand = AutoBuilder.pathfindToPose(autoWayPointConstants.intakeBlueRightPosition,autoWayPointConstants.constraints,0.0); // Goal end velocity in meters/sec);
  Command intakeCenterLeftCommand = AutoBuilder.pathfindToPose(autoWayPointConstants.intakeCenterLeftPosition,autoWayPointConstants.constraints,0.0); // Goal end velocity in meters/sec);
  Command intakeCenterRightCommand = AutoBuilder.pathfindToPose(autoWayPointConstants.intakeCenterRightPosition,autoWayPointConstants.constraints,0.0); // Goal end velocity in meters/sec);
  Command intakeRedLeftCommand = AutoBuilder.pathfindToPose(autoWayPointConstants.intakeRedLeftPosition,autoWayPointConstants.constraints,0.0); // Goal end velocity in meters/sec);
  Command intakeRedightCommand = AutoBuilder.pathfindToPose(autoWayPointConstants.intakeRedRightPosition,autoWayPointConstants.constraints,0.0); // Goal end velocity in meters/sec);

  Command bargeBlueLeftCommand = AutoBuilder.pathfindToPose(autoWayPointConstants.bargeBlueLeftPosition,autoWayPointConstants.constraints,0.0); // Goal end velocity in meters/sec);
  Command bargeBlueRightCommand = AutoBuilder.pathfindToPose(autoWayPointConstants.bargeBlueRightPosition,autoWayPointConstants.constraints,0.0); // Goal end velocity in meters/sec);
  Command bargeCenterLeftCommand = AutoBuilder.pathfindToPose(autoWayPointConstants.bargeCenterLeftPosition,autoWayPointConstants.constraints,0.0); // Goal end velocity in meters/sec);
  Command bargeCenterRightCommand = AutoBuilder.pathfindToPose(autoWayPointConstants.bargeCenterRightPosition,autoWayPointConstants.constraints,0.0); // Goal end velocity in meters/sec);
  Command bargeRedLeftCommand = AutoBuilder.pathfindToPose(autoWayPointConstants.bargeRedLeftPosition,autoWayPointConstants.constraints,0.0); // Goal end velocity in meters/sec);
  Command bargeRedightCommand = AutoBuilder.pathfindToPose(autoWayPointConstants.bargeRedRightPosition,autoWayPointConstants.constraints,0.0); // Goal end velocity in meters/sec);

  public static boolean fieldCentric = true;
  public static boolean reducedSpeed = false;
  public static boolean alignOnLeft = false;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    SmartDashboard.putNumber("elevator P", elevatorConstants.kP);
    SmartDashboard.putNumber("elevator I", elevatorConstants.kI);
    SmartDashboard.putNumber("elevator D", elevatorConstants.kD);

    if(Constants.OperatorConstants.ElevatorSubsystemOnline){
      NamedCommands.registerCommand("LevelFour", eLevelFour);
      NamedCommands.registerCommand("LevelThree", eLevelThree);
      NamedCommands.registerCommand("LevelTwo", eLevelTwo);
      NamedCommands.registerCommand("LevelOne", eLevelOne);
      NamedCommands.registerCommand("L4", autoL4);
      NamedCommands.registerCommand("Move Elevator", runElevatorCommand);

      NamedCommands.registerCommand("eUp", eUp);
      NamedCommands.registerCommand("eDown", eDown);
      NamedCommands.registerCommand("eIntake", eIntake);
    }
    if (Constants.OperatorConstants.coralFeederOnline){
      NamedCommands.registerCommand("IntakeCoral", intakeCoral2);
      NamedCommands.registerCommand("runCoral", autoCoralIntake);
    }
    NamedCommands.registerCommand("reset Gyro", resetGyro);


    NamedCommands.registerCommand("Score0", namedAutoCoral); // ?!?!?!?

    autoChooser = AutoBuilder.buildAutoChooser(); // <

    // Configure the trigger bindings
    configureBindings();
    
    if (Constants.OperatorConstants.driverSystemOnline){
      m_DriveManager.setDefaultCommand(
        new RunCommand( ()-> m_DriveManager.ManageSwerveSystem(
          -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDeadband),
          -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDeadband),
          -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDeadband),
          fieldCentric, false, reducedSpeed),m_DriveManager));
    }
    
    if(Constants.OperatorConstants.ElevatorSubsystemOnline){
    } 
      m_Elevator.setDefaultCommand(
        new RunCommand( ()-> m_Elevator.moveMotors(), m_Elevator)
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


    Trigger aDriverButton = m_driverController.a();

    Trigger bDriverButton = m_driverController.b();

    if (Constants.OperatorConstants.driverSystemOnline){
      Trigger yDriverButton = m_driverController.y();
      yDriverButton.onTrue(resetGyro);
  
      Trigger xDriverButton = m_driverController.x();
      xDriverButton.onTrue(toggleMode);
  
      Trigger leftDriveButton = m_driverController.povLeft();
      leftDriveButton.toggleOnTrue(alignLeft);
  
      Trigger rightDriverButton = m_driverController.povRight();
      rightDriverButton.toggleOnTrue(alignRight);
  
      Trigger upDriverbutton = m_driverController.povUp();
      upDriverbutton.toggleOnTrue(alignCenter);
  
      Trigger LeftDriverTrigger = m_driverController.leftTrigger();
      LeftDriverTrigger.toggleOnTrue(robotCentricC).toggleOnFalse(fieldCentricC);// hold triger, robot is robot centric, let go and it becomes field centric
  
      Trigger rightDriverTrigger = m_driverController.rightTrigger();
      rightDriverTrigger.toggleOnTrue(reduceSpeed).toggleOnFalse(normalSpeed); // when trigger is held, reduce speed to 25 %
    }
    
    if (Constants.OperatorConstants.ElevatorSubsystemOnline){
         
        Trigger aScorerButton = m_scoringController.a();
          aScorerButton.toggleOnTrue(eIntake);

        Trigger yScoreButton = m_scoringController.y();
          yScoreButton.whileTrue(resetEncoder);
    
        Trigger downScorerButton = m_scoringController.povDown();
          downScorerButton.toggleOnTrue(eDown); // make while true for hold and mov
        //downScorerButton.toggleOnFalse(null);
    
        Trigger upScorerButton = m_scoringController.povUp();
          upScorerButton.toggleOnTrue(eUp);

          if (Constants.OperatorConstants.ElevatorOverrideOnline){
            Trigger leftTrigger = m_scoringController.leftTrigger();
            leftTrigger.whileTrue(EOverride).toggleOnFalse(nullE); // override elevator (be aware unless changed, it will move to its old posisitoion)
          }
          
    }
  

    if (Constants.OperatorConstants.coralFeederOnline){
      Trigger rightTrigger = m_scoringController.rightTrigger();

      rightTrigger.whileTrue(intakeCoral);

      Trigger rightBumper = m_scoringController.rightBumper();
      rightBumper.onTrue(intakeCoral2);
    }
        

    if (Constants.OperatorConstants.autoAlignOnline){
      Trigger intakeBlue = buttonPannel.button(1);
      intakeBlue.onTrue(intakeBlueLeftCommand);
  
      Trigger intakeCenter = buttonPannel.button(2);
      intakeCenter.onTrue(intakeCenterLeftCommand);
  
      Trigger intakeRed = buttonPannel.button(3);
      intakeRed.onTrue(intakeRedLeftCommand);
  
      Trigger bargeBlue = buttonPannel.button(4);
      bargeBlue.onTrue(bargeBlueLeftCommand);
  
      Trigger bargeCenter = buttonPannel.button(5);
      bargeCenter.onTrue(bargeCenterLeftCommand);
  
      Trigger bargeRed = buttonPannel.button(6);
      bargeRed.onTrue(bargeRedLeftCommand);
    }
    




    //SmartDashboard.putData("Auto Chooser", autoChooser);
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public Command getAutonomousCommand() { 
    // runs preplanned pathplanner
    //return autoChooser.getSelected();
    // return new PathPlannerAuto("Auto Elevator");
    return autoChooser.getSelected();
  }

  //If it works, pathplanner is the source of the error
  public Command getManualAutonomousCommand() { 
    return
    new ParallelRaceGroup( 
      new RunCommand(() ->m_DriveManager.ManageSwerveSystem(
        -0.5,
        0.0,
        0.0,
        false, false, false),m_DriveManager),
        new WaitCommand(1)
    ); // put robot backwards

  }

  public Command getManualAutoCenterCommand(){ // Auto we want to test
    return Commands.runOnce(()-> m_DriveManager.ManageSwerveSystem(.5, 0, 0, false, false, false), m_DriveManager)
    .andThen(new WaitCommand(1))//second
    .andThen(()-> m_DriveManager.ManageSwerveSystem(0, 0, 0, false, false, false), m_DriveManager)
    .andThen(()-> m_Elevator.setCurrentPosition(Constants.elevatorConstants.levelFourPositionIndex), m_Elevator)
    .andThen(()-> m_Elevator.moveMotors(), m_Elevator)
    .andThen(new WaitCommand(2.5))
    .andThen(()-> m_Coral.intakeMotor(Constants.coralFeederConstants.clawIntakeSpeed), m_Coral)
    .andThen(new WaitCommand(1))
    .andThen(()-> m_Coral.stopMotor(), m_Coral)
    .andThen(()-> m_Elevator.setCurrentPosition(Constants.elevatorConstants.intakePositionIndex), m_Elevator)
    .andThen(()-> m_Elevator.moveMotors(), m_Elevator)
    .andThen(new WaitCommand(2.5))
    .andThen(()-> m_DriveManager.ManageSwerveSystem(-.5, 0, 0, false, false, false), m_DriveManager)
    .andThen(new WaitCommand(.5))
    .andThen(()-> m_DriveManager.ManageSwerveSystem(0, 0, 0, false, false, false), m_DriveManager)
    .andThen(()-> m_DriveManager.ManageSwerveSystem(0, 0, .5, false, false, false), m_DriveManager)
    .andThen(new WaitCommand(1))
    .andThen(()-> m_DriveManager.ManageSwerveSystem(0, 0, 0, false, false, false), m_DriveManager)
    .andThen(()-> m_DriveManager.driveSystem.zeroHeading(),m_DriveManager)
    ;

    /*  REFERNCE
    return Commands.runOnce(
      () -> shooterSubsystem.shootSpeaker(),
      shooterSubsystem).andThen(new WaitCommand(2))
      .andThen(()->intakeSubsystem.enableIntake(0.5),
      intakeSubsystem).andThen(new WaitCommand(5))
      .andThen(new WaitCommand(1)).andThen(()-> shooterSubsystem.endShoot(),shooterSubsystem)
      .andThen(()-> m_robotDrive.drive(0.0,0.5,0.0,OIConstants.fieldRelative,true),m_robotDrive);
      */
      //.andThen(backwards);
    //return Commands.runOnce();
    //return Commands.runOnce(()-> ampArmSubsystem.move(.22).ampArmSubsystem).andThen(new WaitCommand(2)).andThen(()-> ampArmSubsystem.stop().ampArmSubsystem);
  }
  public Command getManualLimelightCommand(){ // Want to use limelight with it
    // parralle race group
    //
    return Commands.runOnce(()-> m_DriveManager.alignOnTag(), m_DriveManager)
    .andThen(new ParallelRaceGroup(new RunCommand(()-> m_DriveManager.alignOnTag(), m_DriveManager).repeatedly(), new WaitCommand(3)))
    .andThen(new ParallelRaceGroup(new RunCommand(()-> m_DriveManager.ManageSwerveSystem(.25, 0, m_DriveManager.rotateToHeading(m_DriveManager.getGyro().getAngle(), m_DriveManager.getDesiredAngle()), false, false, false), m_DriveManager).repeatedly(), new WaitCommand(2)))
    //.andThen(()-> m_DriveManager.ManageSwerveSystem(.25, 0, m_DriveManager.rotateToHeading(m_DriveManager.getGyro().getAngle(), m_DriveManager.getDesiredAngle()), false, false, false), m_DriveManager)
    //.andThen(new WaitCommand(5))
    .andThen(()-> m_DriveManager.ManageSwerveSystem(0, 0, 0, false, false, false), m_DriveManager)
    //.andThen(new WaitCommand(2.5))
    .andThen(()-> m_Elevator.setCurrentPosition(Constants.elevatorConstants.levelFourPositionIndex), m_Elevator)
    .andThen(()-> m_Elevator.moveMotors(), m_Elevator) // can make a race group
    .andThen(new WaitCommand(2.5))
    .andThen(()-> m_Coral.intakeMotor(Constants.coralFeederConstants.clawIntakeSpeed), m_Coral)
    .andThen(new WaitCommand(2))
    .andThen(()-> m_Coral.stopMotor(), m_Coral)
    .andThen(()-> m_Elevator.setCurrentPosition(Constants.elevatorConstants.intakePositionIndex), m_Elevator)
    .andThen(()-> m_Elevator.moveMotors(), m_Elevator) // race group
    .andThen(new WaitCommand(2))
    .andThen(()-> m_Elevator.moveElevator(0), m_Elevator)
    .andThen(()-> m_DriveManager.ManageSwerveSystem(-.25, 0, m_DriveManager.rotateToHeading(m_DriveManager.getGyro().getAngle(), m_DriveManager.getDesiredAngle()), false, false, false), m_DriveManager)
    .andThen(new WaitCommand(1))
    .andThen(()-> m_DriveManager.ManageSwerveSystem(0, 0, 0, false, false, false), m_DriveManager)
    .andThen(()-> m_DriveManager.ManageSwerveSystem(0, 0, m_DriveManager.rotateToHeading(m_DriveManager.getGyro().getAngle(), m_DriveManager.getDesiredAngle()+180), false, false, false), m_DriveManager)
    .andThen(new WaitCommand(1))
    .andThen(()-> m_DriveManager.ManageSwerveSystem(0, 0, 0, false, false, false), m_DriveManager)
    .andThen(()-> m_DriveManager.driveSystem.zeroHeading(),m_DriveManager)
    ;
  }
}