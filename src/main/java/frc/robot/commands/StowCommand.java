package frc.robot.commands;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.algaeendeffector.AlgaeEndEffector;
import frc.robot.subsystems.arm.ArmJoint;
import frc.robot.subsystems.coralendeffector.CoralEndEffector;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.util.LoggedTunableNumber;

public class StowCommand extends SequentialCommandGroup {
    private enum ShoulderPositions {
        Starting(new LoggedTunableNumber("Positions/StowCommand/shoulder/StartingDegrees", 0)),
        SafeToSwingElbow(new LoggedTunableNumber("Positions/StowCommand/shoulder/SafeToSwingElbowDegrees", 105)),
        SafeToSwingElbowHigh(new LoggedTunableNumber("Positions/StowCommand/shoulder/SafeToSwingElbowHighDegrees", -10)),
        Final(new LoggedTunableNumber("Positions/StowCommand/shoulder/FinalDegrees", 90));

        DoubleSupplier position;
        
        MutAngle distance;

        ShoulderPositions(DoubleSupplier position) {
            this.position = position;
            this.distance = Degrees.mutable(0.0);
        }

        public Angle angle() {
            this.distance.mut_replace(this.position.getAsDouble(), Degrees);
            return this.distance;
        }
    }

    private enum ElbowPositions {
        Starting(new LoggedTunableNumber("Positions/StowCommand/elbow/StartingDegrees", 0)),
        Final(new LoggedTunableNumber("Positions/StowCommand/elbow/FinalDegrees", 65));

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

    private enum WristPositions {
        Starting(new LoggedTunableNumber("Positions/StowCommand/wrist/StartingDegrees", 0)),
        Final(new LoggedTunableNumber("Positions/StowCommand/wrist/FinalDegrees", 0));

        DoubleSupplier position;
        MutAngle distance;

        WristPositions(DoubleSupplier position) {
            this.position = position;
            this.distance = Degrees.mutable(0.0);
        }

        public Angle angle() {
            this.distance.mut_replace(this.position.getAsDouble(), Degrees);
            return this.distance;
        }
    }

    private enum ElevatorPositions {
        Starting(new LoggedTunableNumber("Positions/StowCommand/elevator/StartingInches", 0)),
        Final(new LoggedTunableNumber("Positions/StowCommand/elevator/FinalInches",0));

        DoubleSupplier position;
        MutDistance distance;

        ElevatorPositions(DoubleSupplier position) {
            this.position = position;
            this.distance = Inches.mutable(0.0);
        }

        public Distance distance() {
            this.distance.mut_replace(this.position.getAsDouble(), Inches);
            return this.distance;
        }
    }

    public StowCommand(ArmJoint shoulder, ArmJoint elbow, Elevator elevator, Wrist wrist, CoralEndEffector coralEE, AlgaeEndEffector algaeEE) {
        super(
            wrist.getNewWristTurnCommand(WristPositions.Final.position),
            shoulder.getNewSetAngleCommand(ShoulderPositions.Final.position),
            elevator.getNewSetDistanceCommand(ElevatorPositions.Final.distance().in(Inches))
                .alongWith(
                    new WaitUntilCommand(shoulder.getNewLessThanAngleTrigger(ShoulderPositions.SafeToSwingElbow.position)),
                    new WaitUntilCommand(shoulder.getNewGreaterThanAngleTrigger(ShoulderPositions.SafeToSwingElbowHigh.position))
                )
            .andThen(
                elbow.getNewSetAngleCommand(ElbowPositions.Final.position)
            ),
            coralEE.getNewSetVoltsCommand(1)
                .alongWith(algaeEE.getNewSetVoltsCommand(0)),
            new WaitUntilCommand(getNewAtStowTrigger(shoulder, elbow, elevator, wrist)).withTimeout(0.5)
        );
        addRequirements(shoulder, elbow, wrist, elevator, coralEE, algaeEE);
    }

    public static Command getNewUnsafeStowCommand(ArmJoint shoulder, ArmJoint elbow, Elevator elevator, Wrist wrist, CoralEndEffector coralEE, AlgaeEndEffector algaeEE) {
        return new SequentialCommandGroup(
            wrist.getNewWristTurnCommand(WristPositions.Final.position),
            shoulder.getNewSetAngleCommand(ShoulderPositions.Final.position),
            elevator.getNewSetDistanceCommand(ElevatorPositions.Final.position),
            elbow.getNewSetAngleCommand(ElbowPositions.Final.position)

        );
    }

    public static Trigger getNewAtStowTrigger(ArmJoint shoulder, ArmJoint elbow, Elevator elevator, Wrist wrist) {
        return shoulder.getNewAtAngleTrigger(Degrees.of(ShoulderPositions.Final.position.getAsDouble()), Degrees.of(6.0))
            .and(elbow.getNewAtAngleTrigger(Degrees.of(ElbowPositions.Final.position.getAsDouble()), Degrees.of(5.0)))
            .and(elevator.getNewAtDistanceTrigger(Inches.of(ElevatorPositions.Final.position.getAsDouble()), Inches.of(2.0)))
            .and(wrist.getNewAtAngleTrigger(Degrees.of(WristPositions.Final.position.getAsDouble()), Degrees.of(5.0)));
    }
}
