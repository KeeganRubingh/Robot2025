package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.climber.Climber;

public class NeutralClimber extends SequentialCommandGroup {

    
    public NeutralClimber(Climber climber, boolean lockServo) {
        super(
            climber.getNewStopClimberCommand(),
            lockServo ? climber.getNewSetServoAngleCommand(140.0) : new InstantCommand()
        );
    }
}
