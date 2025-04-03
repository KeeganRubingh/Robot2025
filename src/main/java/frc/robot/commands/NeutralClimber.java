package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.climber.Climber;

public class NeutralClimber extends SequentialCommandGroup {

    
    public NeutralClimber(Climber climber) {
        super(
            climber.getNewSetVoltsCommand(0.0),
            climber.getNewSetServoAngleCommand(120.0)
        );
    }
}
