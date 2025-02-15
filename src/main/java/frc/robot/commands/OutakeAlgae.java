package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.algaeendeffector.AlgaeEndEffector;

public class OutakeAlgae extends SequentialCommandGroup {
    public OutakeAlgae(AlgaeEndEffector toesies) {
        super(
            toesies.getNewSetVoltsCommand(-6)
        );
        addRequirements(toesies);
    }
}