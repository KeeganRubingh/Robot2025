package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.coralendeffector.CoralEndEffector;

public class OutakeCoral extends SequentialCommandGroup {    
    public OutakeCoral(CoralEndEffector fingeys) {
        super(
            fingeys.getNewSetVoltsCommand(-6)
        );
        addRequirements(fingeys);
    }
}