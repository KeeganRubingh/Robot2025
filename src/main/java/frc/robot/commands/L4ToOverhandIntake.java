package frc.robot.commands;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.ArmJoint;
import frc.robot.subsystems.coralendeffector.CoralEndEffector;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.util.LoggedTunableNumber;

public class L4ToOverhandIntake extends SequentialCommandGroup {

    private enum ShoulderPositions {
        Final(new LoggedTunableNumber("L4ToOverhandIntake/shoulder/FinalDegrees", -150.0));

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
        Final(new LoggedTunableNumber("L4ToOverhandIntake/elbow/FinalDegrees", -40.0));

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
        Final(new LoggedTunableNumber("L4ToOverhandIntake/wrist/FinalDegrees", -90.0));

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
        Final(new LoggedTunableNumber("L4ToOverhandIntake/elevator/FinalInches", 4.0));

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

    /**
     * Regular StowToL4 using both Elevator and Arm
     */
    public L4ToOverhandIntake(ArmJoint shoulder, ArmJoint elbow, Elevator elevator, Wrist wrist, CoralEndEffector coralEE) {
        super(
            shoulder.getNewSetAngleCommand(ShoulderPositions.Final.position),
            elbow.getNewSetAngleCommand(ElbowPositions.Final.position),
            wrist.getNewWristTurnCommand(WristPositions.Final.position),
            elevator.getNewSetDistanceCommand(ElevatorPositions.Final.position),
            coralEE.getNewSetVoltsCommand(6.0)
        );
        addRequirements(shoulder, elbow, wrist, elevator);
    }

}
