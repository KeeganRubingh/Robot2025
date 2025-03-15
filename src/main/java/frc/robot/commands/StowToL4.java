package frc.robot.commands;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.arm.ArmJoint;
import frc.robot.subsystems.coralendeffector.CoralEndEffector;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.util.LoggedTunableNumber;

public class StowToL4 extends SequentialCommandGroup {

    private enum ShoulderPositions {
        Starting(new LoggedTunableNumber("MoveToL4Command/shoulder/StartingDegrees", 95.0)),
        SafeToSwingElbow(new LoggedTunableNumber("MoveToL4Command/shoulder/SafeToSwingElbowDegrees", -20)),
        Final(new LoggedTunableNumber("MoveToL4Command/shoulder/FinalDegrees", -55));

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
        Starting(new LoggedTunableNumber("MoveToL4Command/elbow/StartingDegrees", 65)),
        MidPoint(new LoggedTunableNumber("MoveToL4Command/elbow/ShoulderSafeSwingDegrees", 0)),
        Final(new LoggedTunableNumber("MoveToL4Command/elbow/FinalDegrees", -100)),
        Confirm(new LoggedTunableNumber("MoveToL4Command/elbow/ConfirmDegrees", -140));

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
        Starting(new LoggedTunableNumber("MoveToL4Command/wrist/StartingDegrees", 0)),
        Final(new LoggedTunableNumber("MoveToL4Command/wrist/FinalDegrees", 0));

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
        Starting(new LoggedTunableNumber("MoveToL4Command/elevator/StartingInches", 0)),
        SafeToSwingShoulder(new LoggedTunableNumber("MoveToL4Command/elevator/SafeToSwingShoulderInches", 5.0)),
        Final(new LoggedTunableNumber("MoveToL4Command/elevator/FinalInches", 16));

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

    public StowToL4(ArmJoint shoulder, ArmJoint elbow, Elevator elevator, Wrist wrist) {
        super(
            new WaitUntilCommand(shoulder.getNewLessThanAngleTrigger(ShoulderPositions.Starting.position)),
            wrist.getNewWristTurnCommand(WristPositions.Final.position),
            elevator.getNewSetDistanceCommand(ElevatorPositions.Final.position)
                .alongWith(
                    new WaitUntilCommand(elevator.getNewGreaterThanDistanceTrigger(ElevatorPositions.SafeToSwingShoulder.position))
                ),
            shoulder.getNewSetAngleCommand(ShoulderPositions.Final.position),
            elbow.getNewSetAngleCommand(ElbowPositions.MidPoint.position)
                .andThen(
                    new WaitUntilCommand(shoulder.getNewLessThanAngleTrigger(ShoulderPositions.SafeToSwingElbow.position))
                        .andThen(
                            elbow.getNewSetAngleCommand(ElbowPositions.Final.position)
                        )
                        .andThen(
                            new WaitUntilCommand(elbow.getNewLessThanAngleTrigger(ElbowPositions.Final.position.getAsDouble() + 5.0))
                        )
                )
        );
        addRequirements(shoulder, elbow, wrist, elevator);
    }

    public static Command getNewScoreCommand(ArmJoint elbow, Wrist wrist, CoralEndEffector coralEndEffector) {
        return(
            elbow.getNewSetAngleCommand(ElbowPositions.Confirm.position)
            .alongWith(wrist.getNewApplyCoastModeCommand())
            .alongWith(new WaitCommand(0.25))
        )
        .andThen(coralEndEffector.getNewSetVoltsCommand(-4))
        .andThen(new WaitCommand(0.25));
    }

    public static Command getNewStopScoreCommand(ArmJoint elbow, Wrist wrist, CoralEndEffector coralEndEffector, Drive drive) {
        return
        DriveCommands.joystickForwardDrive(drive, () -> 0.5, () -> 0.0, null)
            .withTimeout(1.0)
        .andThen(wrist.getNewWristTurnCommand(0)
            .alongWith(elbow.getNewSetAngleCommand(ElbowPositions.Final.position))
        )
        .andThen(coralEndEffector.getNewSetVoltsCommand(1));
    }

}
