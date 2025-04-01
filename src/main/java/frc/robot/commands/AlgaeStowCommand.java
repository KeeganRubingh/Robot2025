package frc.robot.commands;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.algaeendeffector.AlgaeEndEffector;
import frc.robot.subsystems.arm.ArmJoint;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intakeextender.IntakeExtender;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.util.LoggedTunableNumber;

public class AlgaeStowCommand extends SequentialCommandGroup {

    

    private enum ShoulderPositions {
        Final(new LoggedTunableNumber("Positions/AlgaeStowCommand/shoulder/FinalDegrees", 71.0));

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
        Final(new LoggedTunableNumber("Positions/AlgaeStowCommand/elbow/FinalDegrees", 198));

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

    private enum ElevatorPositions {
        Final(new LoggedTunableNumber("Positions/AlgaeStowCommand/elevator/FinalInches", 0));

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

    private enum WristPositions {
        Final(new LoggedTunableNumber("Positions/AlgaeStowCommand/wrist/FinalDegrees", 0));

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

    private static enum IntakeExtenderPositions{
        Stow(new LoggedTunableNumber("Positions/AlgaeStowCommand/IntakeExtender/StowDegrees",StowToGroundIntake.intakeExtenderStow.in(Degrees))),
        OutOfTheWay(new LoggedTunableNumber("Positions/AlgaeStowCommand/IntakeExtender/OutOfTheWay",-75.0)),
        StowSafeZone(new LoggedTunableNumber("Positions/AlgaeStowCommand/IntakeExtender/StowSafeZoneDegrees",StowToGroundIntake.intakeExtenderStow.in(Degrees) - 2.0));

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

    public AlgaeStowCommand(ArmJoint shoulder, ArmJoint elbow, Elevator elevator, Wrist wrist, AlgaeEndEffector algaeEE, IntakeExtender extender) {
        super(
            extender.getNewIntakeExtenderTurnCommand(IntakeExtenderPositions.Stow.position),
            wrist.getNewWristTurnCommand(WristPositions.Final.position),
            elbow.getNewSetAngleCommand(ElbowPositions.Final.position),
            shoulder.getNewSetAngleCommand(ShoulderPositions.Final.position),
            elevator.getNewSetDistanceCommand(ElevatorPositions.Final.position),
            algaeEE.getNewSetVoltsCommand(2.0)
        );
        addRequirements(shoulder, elbow,  elevator, wrist, algaeEE, extender);
    }
}
