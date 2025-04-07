package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.algaeendeffector.AlgaeEndEffector;

public class BargeScoreCommand extends SequentialCommandGroup {

    
    public BargeScoreCommand(AlgaeEndEffector algaeEE) {
        super(
            algaeEE.getNewSetVoltsCommand(-12.0)
        );
        addRequirements(algaeEE);
    }
}