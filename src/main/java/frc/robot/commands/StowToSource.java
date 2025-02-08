package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.ArmJoint;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.fingeys.Fingeys;
import frc.robot.subsystems.wrist.Wrist;


public class StowToSource extends SequentialCommandGroup {
    public ArmJoint shoulder;
    public ArmJoint elbow;
    public Elevator elevator;
    public Wrist wrist;
    public Fingeys fingeys;

    public StowToSource(ArmJoint shoulder, ArmJoint elbow, Wrist wrist, Fingeys fingeys) {
        this.shoulder = shoulder;
        this.elbow = elbow;
        this.wrist = wrist;
        this.fingeys = fingeys;
    }


    
}
