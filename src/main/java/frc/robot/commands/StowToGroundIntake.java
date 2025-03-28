package frc.robot.commands;
import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.Degrees;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.arm.ArmJoint;
import frc.robot.subsystems.coralendeffector.CoralEndEffector;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intakeextender.IntakeExtender;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.util.LoggedTunableNumber;


public class StowToGroundIntake extends SequentialCommandGroup {
    private static enum ShoulderPositions {
        Starting(new LoggedTunableNumber("StowToGroundIntake/Shoulder/StartingDegrees", 90)),
        ExtendSafe(new LoggedTunableNumber("StowToGroundIntake/Shoulder/ExtendSafeDegrees", 45.5)),
        Final(new LoggedTunableNumber("StowToGroundIntake/Shoulder/FinalDegrees", 55.5));
        

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

    private static enum ElbowPositions {
        Starting(new LoggedTunableNumber("StowToGroundIntake/Elbow/StartingDegrees",65)),
        Final(new LoggedTunableNumber("StowToGroundIntake/Elbow/FinalDegrees", 158));

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

    private static enum WristPositions {
        Starting(new LoggedTunableNumber("StowToGroundIntake/Wrist/StartingDegrees", 0)),
        Final(new LoggedTunableNumber("StowToGroundIntake/Wrist/FinalDegrees", -90));

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

    public static final Angle intakeExtenderStow = Degrees.of(-45.0);
    public static final Angle intakeExtenderTransfer = Degrees.of(0.0);
    public static final Angle intakeExtenderFinal = Degrees.of(-155.0);
    
    private static enum IntakeExtenderPositions {
        Stow(new LoggedTunableNumber("StowToGroundIntake/IntakeExtender/StowDegrees",intakeExtenderStow.in(Degrees))),
        Transfer(new LoggedTunableNumber("StowToGroundIntake/IntakeExtender/TransferDegrees", intakeExtenderTransfer.in(Degrees))),
        Final(new LoggedTunableNumber("StowToGroundIntake/IntakeExtender/FinalDegrees", intakeExtenderFinal.in(Degrees)));

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



    /**
     * The initial stow to ground intake command. Prepares the arm to take from the ground intake
     * @param shoulder
     * @param elbow
     * @param wrist
     * @param fingeys
     */
    public StowToGroundIntake(ArmJoint shoulder, ArmJoint elbow, Wrist wrist, CoralEndEffector fingeys) {
        addRequirements(shoulder, elbow, wrist, fingeys);
        addCommands(
            elbow.getNewSetAngleCommand(ElbowPositions.Final.position),
            shoulder.getNewSetAngleCommand(ShoulderPositions.Final.position),
            wrist.getNewWristTurnCommand(WristPositions.Final.position)
        );
    }

    /**
     * The second part of the ground intake. Spins up the ground intake.
     * If the ground intake has a coral, moves it to the ready pos.
     * @return
     */
    public static Command getRunGroundIntakeCommand(Intake intake, IntakeExtender extender) {
        return intake.getNewSetVoltsCommand(4)
                .alongWith(extender.getNewIntakeExtenderTurnCommand(-155.0))
                .alongWith(new WaitUntilCommand(extender.getNewAtAngleTrigger(()->-155.0, Degrees.of(10))))
                .andThen(new WaitUntilCommand(intake.hasCoralTrigger()))
                .andThen(intake.getNewSetVoltsCommand(2.0))
                .andThen(extender.getNewIntakeExtenderTurnCommand(IntakeExtenderPositions.Stow.position));
    }

    /**
     * The third part of the ground intake. Raises the ground intake, takes a coral from it, then moves the extender to the intake pos
     * @return
     */
    public static Command getTakeCoralFromGroundIntakeCommand(Intake intake, IntakeExtender extender, ArmJoint shoulder, ArmJoint elbow, Wrist wrist, CoralEndEffector coralEndEffector) {
        return 
        wrist.getNewWristTurnCommand(WristPositions.Final.position)
        .andThen(new WaitUntilCommand(wrist.getNewAtAngleTrigger(Degrees.of(WristPositions.Final.position.getAsDouble()), Degrees.of(3))))
        .andThen(
            coralEndEffector.getNewSetVoltsCommand(6)
            .alongWith(extender.getNewIntakeExtenderTurnCommand(IntakeExtenderPositions.Transfer.position.getAsDouble()))
        )
        .andThen(new WaitUntilCommand(extender.getNewAtAngleTrigger(IntakeExtenderPositions.Transfer.position, Degrees.of(3))))
        .andThen(intake.getNewSetVoltsCommand(-5))
        .andThen(new WaitUntilCommand(coralEndEffector.hasCoralTrigger())) 
        .andThen(intake.getNewSetVoltsCommand(0))
        .andThen(shoulder.getNewSetAngleCommand(ShoulderPositions.ExtendSafe.position))
        .andThen(new WaitUntilCommand(shoulder.getNewAtAngleTrigger(ShoulderPositions.ExtendSafe.position, ()->5.0)))
        .andThen(
            extender.getNewIntakeExtenderTurnCommand(IntakeExtenderPositions.Stow.position)
            .alongWith(coralEndEffector.getNewSetVoltsCommand(1))
        );
    }

    /**
     * The final part of the ground intake. Returns the arm (With the coral) to the stow pos
     * @param shoulder
     * @param elbow
     * @param wrist
     * @param fingeys
     * @return
     */
    public static Command getReturnToStowCommand(ArmJoint shoulder, ArmJoint elbow, Wrist wrist, CoralEndEffector fingeys, IntakeExtender extender, Intake intake) {
        return extender.getNewIntakeExtenderTurnCommand(IntakeExtenderPositions.Stow.position)
        .andThen(intake.getNewSetSpeedCommand(0))
        .andThen(new WaitUntilCommand(extender.getNewAtAngleTrigger(IntakeExtenderPositions.Stow.position, Degrees.of(5))))
        .andThen(wrist.getNewWristTurnCommand(WristPositions.Starting.position))
        .andThen(new WaitUntilCommand(wrist.getNewAtAngleTrigger(WristPositions.Starting.position, Degrees.of(2))))
        .andThen(shoulder.getNewSetAngleCommand(ShoulderPositions.Starting.position))
        .andThen(elbow.getNewSetAngleCommand(ElbowPositions.Starting.position));
    }
}