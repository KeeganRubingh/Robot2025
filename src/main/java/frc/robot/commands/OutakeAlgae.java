package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.algaeendeffector.AlgaeEndEffector;

public class OutakeAlgae extends SequentialCommandGroup {

    
    public OutakeAlgae(AlgaeEndEffector algaeEE) {
        super(
            algaeEE.getNewSetVoltsCommand(-6.0)
        );
        addRequirements(algaeEE);
    }
}