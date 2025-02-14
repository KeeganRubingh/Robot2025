package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class AutoCommandManager {

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  public AutoCommandManager(Drive drive) {
    configureNamedCommands();
    
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

  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  private void configureNamedCommands() {
    NamedCommands.registerCommand("groundIntake", new PrintCommand("Intaking sir!"));
    NamedCommands.registerCommand("L1Score", new PrintCommand("Scoring sir!"));
  }
}
