package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.climber.Climber;

public class DisengageClimber extends SequentialCommandGroup {
    public DisengageClimber(Climber climber) {
        super(
            climber.getNewSetServoAngleCommand(90.0),
            new WaitCommand(0.1),
            climber.getNewSetVoltsCommand(-3.0)
        );
    }
}
