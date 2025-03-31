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
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.algaeendeffector.AlgaeEndEffector;
import frc.robot.subsystems.arm.ArmJoint;
import frc.robot.subsystems.coralendeffector.CoralEndEffector;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intakeextender.IntakeExtender;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.util.LoggedTunableNumber;

public class StowCommand extends SequentialCommandGroup {

    private enum ShoulderPositions {
        Starting(new LoggedTunableNumber("StowCommand/shoulder/StartingDegrees", 0)),
        SafeToSwingElbow(new LoggedTunableNumber("StowCommand/shoulder/SafeToSwingElbowDegrees", 105)),
        SafeToSwingElbowHigh(new LoggedTunableNumber("StowCommand/shoulder/SafeToSwingElbowHighDegrees", -10)),
        Final(new LoggedTunableNumber("StowCommand/shoulder/FinalDegrees", 90));

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
        Starting(new LoggedTunableNumber("StowCommand/elbow/StartingDegrees", 0)),
        Final(new LoggedTunableNumber("StowCommand/elbow/FinalDegrees", 65));

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
        Starting(new LoggedTunableNumber("StowCommand/wrist/StartingDegrees", 0)),
        Final(new LoggedTunableNumber("StowCommand/wrist/FinalDegrees", 0));

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
        Starting(new LoggedTunableNumber("StowCommand/elevator/StartingInches", 0)),
        Final(new LoggedTunableNumber("StowCommand/elevator/FinalInches",0));

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

    private static enum IntakeExtenderPositions{
        Stow(new LoggedTunableNumber("StowCommand/IntakeExtender/StowDegrees",StowToGroundIntake.intakeExtenderStow.in(Degrees))),
        OutOfTheWay(new LoggedTunableNumber("StowCommand/IntakeExtender/OutOfTheWay",-75.0)),
        StowSafeZone(new LoggedTunableNumber("StowCommand/IntakeExtender/StowSafeZoneDegrees",StowToGroundIntake.intakeExtenderStow.in(Degrees) - 2.0));

        DoubleSupplier position;
        MutAngle distance;

        IntakeExtenderPositions(DoubleSupplier position) {
            this.position = position;
            this.distance = Degrees.mutable(0.0);
        }

        public Angle angle() {
            this.distance.mut_replace(this.position.getAsDouble(), Degrees);
            return this.distance;
        }
    }

    public StowCommand(ArmJoint shoulder, ArmJoint elbow, Elevator elevator, Wrist wrist, CoralEndEffector coralEE, AlgaeEndEffector algaeEE,IntakeExtender extender) {
        super(
            extender.getNewIntakeExtenderTurnCommand(IntakeExtenderPositions.OutOfTheWay.position),
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
            new WaitUntilCommand(getNewAtStowTrigger(shoulder, elbow, elevator, wrist)).withTimeout(0.5),
            extender.getNewIntakeExtenderTurnCommand(IntakeExtenderPositions.Stow.position)
        );
        addRequirements(shoulder, elbow, wrist, elevator, coralEE, algaeEE);
    }

    public static Trigger getNewAtStowTrigger(ArmJoint shoulder, ArmJoint elbow, Elevator elevator, Wrist wrist) {
        return shoulder.getNewAtAngleTrigger(Degrees.of(ShoulderPositions.Final.position.getAsDouble()), Degrees.of(6.0))
            .and(elbow.getNewAtAngleTrigger(Degrees.of(ElbowPositions.Final.position.getAsDouble()), Degrees.of(5.0)))
            .and(elevator.getNewAtDistanceTrigger(Inches.of(ElevatorPositions.Final.position.getAsDouble()), Inches.of(2.0)))
            .and(wrist.getNewAtAngleTrigger(Degrees.of(WristPositions.Final.position.getAsDouble()), Degrees.of(5.0)));
    }
}
