package frc.robot.commands;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.algaeendeffector.AlgaeEndEffector;
import frc.robot.subsystems.arm.ArmJoint;
import frc.robot.subsystems.coralendeffector.CoralEndEffector;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intakeextender.IntakeExtender;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.util.LoggedTunableNumber;

public class L4ToStow extends SequentialCommandGroup {

    

    private enum ElbowPositions {
        SafeToSwingShoulder(new LoggedTunableNumber("Positions/StowCommand/elbow/L4ToStowSafeToSwingShoulder", -10)),
        MidPoint(new LoggedTunableNumber("Positions/StowCommand/elbow/L4ToStowMidpoint", 0));

        DoubleSupplier position;
        MutAngle distance;

        ElbowPositions(DoubleSupplier position) {
            this.position = position;
            this.distance = Degrees.mutable(0.0);
        }

        public Angle angle() {
            this.distance.mut_replace(this.position.getAsDouble(), Degrees);
            return this.distance;
        }
    }

    public L4ToStow(ArmJoint shoulder, ArmJoint elbow, Elevator elevator, Wrist wrist, CoralEndEffector coralEE, AlgaeEndEffector algaeEE, IntakeExtender intakeExtender) {
        super(
            elbow.getNewSetAngleCommand(ElbowPositions.MidPoint.position)
                .alongWith(
                    new WaitUntilCommand(elbow.getNewGreaterThanAngleTrigger(ElbowPositions.SafeToSwingShoulder.position)),
                    elevator.getNewSetDistanceCommand(0.0)
                ),
            new StowCommand(shoulder, elbow, elevator, wrist, coralEE, algaeEE, intakeExtender)
        );
        addRequirements(shoulder, elbow, wrist, elevator, coralEE, algaeEE);
    }
}
