package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import java.util.function.DoubleSupplier;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.arm.ArmJoint;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.fingeys.Fingeys;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.util.LoggedTunableNumber;

public class StowToL3 extends SequentialCommandGroup {
    public ArmJoint shoulder;
    public ArmJoint elbow;
    public Wrist wrist;
    public Fingeys fingeys;

    private enum ShoulderPositions {
        Starting(new LoggedTunableNumber("MoveToL4Command/shoulder/StartingDegrees", 0)),
        MidPoint(new LoggedTunableNumber("MoveToL4Command/shoulder/MidPointDegrees", 110)),
        SafeToSwingElbow(new LoggedTunableNumber("MoveToL4Command/shoulder/SafeToSwingElbowDegrees", 100)),
        Final(new LoggedTunableNumber("MoveToL4Command/shoulder/FinalDegrees", 90));

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
        Starting(new LoggedTunableNumber("MoveToL4Command/elbow/StartingDegrees", 0)),
        ShoulderSafeSwing(new LoggedTunableNumber("MoveToL4Command/elbow/ShoulderSafeSwingDegrees", 45)),
        Final(new LoggedTunableNumber("MoveToL4Command/elbow/FinalDegrees", 90));

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
        Final(new LoggedTunableNumber("MoveToL4Command/wrist/FinalDegrees", 90));

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

    public StowToL3(ArmJoint shoulder, ArmJoint elbow, Wrist wrist, Fingeys fingeys) {
        super(
            wrist.getNewWristTurnCommand(WristPositions.Final.angle().in(Degrees)),
            shoulder.getNewSetAngleCommand(ShoulderPositions.MidPoint.angle().in(Degrees))
                .raceWith(
                    new WaitUntilCommand(shoulder.getNewGreaterThanAngleTrigger(ShoulderPositions.SafeToSwingElbow.angle().in(Degrees)))
                        .andThen(
                            elbow.getNewSetAngleCommand(ElbowPositions.Final.angle().in(Degrees))
                                .raceWith(new WaitUntilCommand(elbow.getNewGreaterThanAngleTrigger(ElbowPositions.ShoulderSafeSwing.angle().in(Degrees)))
                        )
                    )
                ),
            shoulder.getNewSetAngleCommand(ShoulderPositions.Final.angle().in(Degrees))
        );
        addRequirements(shoulder, elbow, wrist, fingeys);
    }
}
