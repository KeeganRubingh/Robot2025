package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.StopDrivetrainCommand;
import frc.robot.subsystems.drive.Drive;

public class AutoCommandManager {

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  public AutoCommandManager(Drive drive) {
    configureNamedCommands(drive);
    

    PathPlannerAuto CharacterizationTest = new PathPlannerAuto("CharacterizeAuto");
    PathPlannerAuto ChoreoStraightAuto = new PathPlannerAuto("ChoreoStraightAuto");

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    PathPlannerAuto Circle = new PathPlannerAuto("Circle");
    PathPlannerAuto Straight = new PathPlannerAuto("Straight");
    PathPlannerAuto Diagonal = new PathPlannerAuto("Diagonal");
    PathPlannerAuto simpleAuto = new PathPlannerAuto("simpleAuto");
    PathPlannerAuto advancedPath = new PathPlannerAuto("advancedPath");
    PathPlannerAuto exitZone = new PathPlannerAuto("exitZone");

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption("Circle", Circle);
    autoChooser.addOption("Straight", Straight);
    autoChooser.addOption("Diagonal", Diagonal);
    autoChooser.addOption("simpleAuto", simpleAuto);
    autoChooser.addOption("advancedPath", advancedPath);
    autoChooser.addOption("exitZone", exitZone);


    // Add Choreo Paths
    autoChooser.addOption("Characterization Test Path", CharacterizationTest);
    autoChooser.addOption("[Choreo] Straight Test", ChoreoStraightAuto);
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  private void configureNamedCommands(Drive drive) {
      NamedCommands.registerCommand("Intake", new PrintCommand("***********Intaking sir!"));
      NamedCommands.registerCommand("StopIntake", new PrintCommand("stop intake sir!"));
      NamedCommands.registerCommand("StartIntake", new PrintCommand("Start intake sir!"));
      NamedCommands.registerCommand("L4Score", new PrintCommand("Scoring L4 sir!"));
      NamedCommands.registerCommand("L1Score", new PrintCommand("Scoring sir!"));
      NamedCommands.registerCommand("L2Score", new PrintCommand("***************Scoring L2 sir!"));
      NamedCommands.registerCommand("StopDrivetrain", new PrintCommand("STOPPING THE DRIVETRAIN SIR**************"));
  }
}
