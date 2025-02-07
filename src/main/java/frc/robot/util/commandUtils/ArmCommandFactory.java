package frc.robot.util.commandUtils;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.arm.ArmJoint;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.fingeys.Fingeys;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.toesies.Toesies;
import frc.robot.subsystems.wrist.Wrist;

public class ArmCommandFactory {

    private final RobotSystems RS;

    public ArmCommandFactory() {
        RS = RobotSystems.getInstance();
    }

    public Command getArmIntakeToStowCommand() {
        return 
        RS.wrist.getNewWristTurnCommand(0).alongWith(new WaitUntilCommand(RS.wrist.getNewAtSetpointTrigger()))
        .andThen(RS.elbow.getNewSetAngleCommand(-15).alongWith(new WaitUntilCommand(RS.elbow.getNewAtSetpointTrigger())))
        .andThen(RS.shoulder.getNewSetAngleCommand(-135).alongWith(new WaitUntilCommand(RS.wrist.getNewAtSetpointTrigger())))
        .andThen(RS.elbow.getNewSetAngleCommand(-15).alongWith(new WaitUntilCommand(RS.elbow.getNewAtSetpointTrigger())))
        .andThen(RS.);
    }
}