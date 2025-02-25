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
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.util.LoggedTunableNumber;

public class StationIntakeToStow extends SequentialCommandGroup {

    private enum WristPositions {
        SafeToSwingShoulder(new LoggedTunableNumber("StationIntakeToStow/wrist/SafeToSwingShoulder", -20.0)),
        Final(new LoggedTunableNumber("StationIntakeToStow/wrist/FinalDegrees", 0));

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

    public StationIntakeToStow(ArmJoint shoulder, ArmJoint elbow, Elevator elevator, Wrist wrist, CoralEndEffector coralEE, AlgaeEndEffector algaeEE) {
        super(
            wrist.getNewWristTurnCommand(WristPositions.Final.position),
            // Needed so coral doesn't hit bar on way back
            new WaitUntilCommand(wrist.getNewGreaterThanAngleTrigger(WristPositions.SafeToSwingShoulder.position)),
            new StowCommand(shoulder, elbow, elevator, wrist, coralEE, algaeEE)
        );
        addRequirements(shoulder, elbow, wrist, elevator, coralEE, algaeEE);
    }
}
