package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.arm.ArmJoint;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.fingeys.Fingeys;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intakeextender.IntakeExtender;
import frc.robot.subsystems.toesies.Toesies;
import frc.robot.subsystems.wrist.Wrist;
import pabeles.concurrency.ConcurrencyOps.NewInstance;

public final class CommandFactory {
    private static CommandFactory instance;

    private ArmJoint shoulder;
    private ArmJoint elbow;
    private Elevator elevator;
    private Fingeys fingeys;
    private Intake intake;
    private IntakeExtender extender;
    private Toesies toesies;
    private Wrist wrist;
    
    public static void initialize(ArmJoint shoulder, ArmJoint elbow, Elevator elevator, Fingeys fingeys, Intake intake, IntakeExtender extender, Toesies toesies, Wrist wrist) {
        instance = new CommandFactory(shoulder, elbow, elevator, fingeys, intake, extender, toesies, wrist);
    }
    public static CommandFactory getInstance() {
        if(instance == null) {
            DriverStation.reportError("LMAO COMMAND FACTORY BROKE", true);
            return null;
        }

        return instance;
    }
    private CommandFactory(ArmJoint shoulder, ArmJoint elbow, Elevator elevator, Fingeys fingeys, Intake intake, IntakeExtender extender, Toesies toesies, Wrist wrist) {
        this.shoulder = shoulder;
        this.elbow = elbow;
        this.elevator = elevator;
        this.fingeys = fingeys;
        this.intake = intake;
        this.extender = extender;
        this.toesies = toesies;
        this.wrist = wrist;
    }

    public Command getNewStartIntakeCommand(){
        return intake.getNewSetSpeedCommand(1.0)
        .alongWith(extender.getNewIntakeExtenderTurnCommand(90))
        .alongWith(new WaitCommand(5))
        .andThen(extender.getNewIntakeExtenderTurnCommand(0))
        .andThen(intake.getNewSetSpeedCommand(0.0));
    }
}
