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
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.algaeendeffector.AlgaeEndEffector;
import frc.robot.subsystems.arm.ArmJoint;
import frc.robot.subsystems.coralendeffector.CoralEndEffector;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.util.LoggedTunableNumber;

public class ReadyProcessorScore extends SequentialCommandGroup {
    
    private enum ShoulderPositions {
        Final(new LoggedTunableNumber("ReadyProcessorScore/shoulder/FinalDegrees", 20.0)),
        Tolerance(new LoggedTunableNumber("ReadyProcessorScore/shoulder/Tolerance", 0.5));

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
        Final(new LoggedTunableNumber("ReadyProcessorScore/elbow/FinalDegrees", 70.0)),
        Tolerance(new LoggedTunableNumber("ReadyProcessorScore/shoulder/Tolerance", 0.5));

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
        Starting(new LoggedTunableNumber("ReadyProcessorScore/elevator/StartingInches", 0)),
        Final(new LoggedTunableNumber("ReadyProcessorScore/elevator/FinalInches", 0));

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
        Starting(new LoggedTunableNumber("ReadyProcessorScore/wrist/StartingDegrees", 0)),
        Final(new LoggedTunableNumber("ReadyProcessorScore/wrist/FinalDegrees", 0));

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

    public ReadyProcessorScore(ArmJoint shoulder, ArmJoint elbow, Elevator elevator, Wrist wrist, AlgaeEndEffector algaeEE) {
        super(
            wrist.getNewWristTurnCommand(WristPositions.Final.position),
            new InstantCommand(()->{
                System.out.println("Test");
            }),
            elbow.getNewSetAngleCommand(ElbowPositions.Final.position),
            shoulder.getNewSetAngleCommand(ShoulderPositions.Final.position),
            new WaitUntilCommand(shoulder.getNewAtAngleTrigger(ShoulderPositions.Final.position,ShoulderPositions.Tolerance.position)),
            new InstantCommand(()->{
                System.out.println("Test");
            }),
            new WaitUntilCommand(elbow.getNewAtAngleTrigger(ElbowPositions.Final.position,ElbowPositions.Tolerance.position)),
            new InstantCommand(()->{
                System.out.println("Test");
            }),
            algaeEE.getNewSetVoltsCommand(4.0)
            
        );
        addRequirements(shoulder, elbow, elevator, wrist, algaeEE);
    }
}
