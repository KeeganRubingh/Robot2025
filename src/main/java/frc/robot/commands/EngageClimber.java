package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.climber.Climber;

public class EngageClimber extends SequentialCommandGroup {

    
    public EngageClimber(Climber climber) {
        super(
            climber.getNewSetServoAngleCommand(0.0),
            new WaitCommand(0.1),
            climber.getNewSetVoltsCommand(3.0) // negative was forward in test
        );
    }
}
