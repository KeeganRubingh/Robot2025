package frc.robot.util;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.StowToL1;
import frc.robot.commands.StowToL2;
import frc.robot.commands.StowToL3;
import frc.robot.commands.StowToL4;
import frc.robot.subsystems.arm.ArmJoint;
import frc.robot.subsystems.coralendeffector.CoralEndEffector;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.util.ReefPositionsUtil.ScoreLevel;

public class SelectorCommandFactory {
    public static Map<ScoreLevel,Command> getCoralLevelPrepCommandSelector(ArmJoint shoulder,ArmJoint elbow,Elevator elevator,Wrist wrist) {
        Map<ScoreLevel,Command> commandMap = new HashMap<>();
        commandMap.put(ScoreLevel.L1, new StowToL1(shoulder, elbow, wrist));
        commandMap.put(ScoreLevel.L2, new StowToL2(shoulder, elbow, elevator, wrist));
        commandMap.put(ScoreLevel.L3, new StowToL3(shoulder, elbow, wrist, elevator));
        commandMap.put(ScoreLevel.L4, new StowToL4(shoulder, elbow, elevator, wrist));
        return commandMap;
    }

    public static Map<ScoreLevel,Command> getCoralLevelScoreCommandSelector(ArmJoint shoulder, ArmJoint elbow, Elevator elevator, Wrist wrist,CoralEndEffector coralEndEffector) {
        Map<ScoreLevel,Command> commandMap = new HashMap<>();
        commandMap.put(ScoreLevel.L1, StowToL1.getNewScoreCommand(coralEndEffector));
        commandMap.put(ScoreLevel.L2, StowToL2.getNewScoreCommand(shoulder, elbow, wrist, coralEndEffector));
        commandMap.put(ScoreLevel.L3, StowToL3.getNewScoreCommand(shoulder, elbow, wrist, coralEndEffector));
        commandMap.put(ScoreLevel.L4, StowToL4.getNewScoreCommand(elbow, wrist, coralEndEffector));
        return commandMap;
    }

    public static Map<ScoreLevel,Command> getCoralLevelStopScoreCommandSelector(ArmJoint elbow, Wrist wrist, CoralEndEffector coralEndEffector, Drive drive) {
        Map<ScoreLevel,Command> commandMap = new HashMap<>();
        commandMap.put(ScoreLevel.L1, StowToL1.getNewStopScoreCommand(coralEndEffector));
        commandMap.put(ScoreLevel.L2, StowToL2.getNewStopScoreCommand(elbow, wrist, coralEndEffector));
        commandMap.put(ScoreLevel.L3, StowToL3.getNewStopScoreCommand(elbow, wrist, coralEndEffector));
        commandMap.put(ScoreLevel.L4, StowToL4.getNewStopScoreCommand(elbow, wrist, coralEndEffector, drive));
        return commandMap;
    }

    public static Map<ScoreLevel,Command> getCoralLevelWaitUntilAtLevelCommandSelector(ArmJoint shoulder, ArmJoint elbow, Elevator elevator, Wrist wrist) {
        Map<ScoreLevel,Command> commandMap = new HashMap<>();
        commandMap.put(ScoreLevel.L1, new WaitUntilCommand(StowToL1.getNewAtScoreTrigger(shoulder, elbow, wrist)));
        commandMap.put(ScoreLevel.L2, new WaitUntilCommand(StowToL2.getNewAtScoreTrigger(shoulder, elbow, wrist)));
        commandMap.put(ScoreLevel.L3, new WaitUntilCommand(StowToL3.getNewAtScoreTrigger(shoulder, elbow, elevator, wrist)));
        commandMap.put(ScoreLevel.L4, new WaitUntilCommand(StowToL4.getNewAtScoreTrigger(shoulder, elbow, elevator, wrist)));
        return commandMap;
    }
}