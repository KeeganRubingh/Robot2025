package frc.robot.commands;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static frc.robot.subsystems.vision.VisionConstants.aprilTagLayout;

import java.util.Map;
import java.util.function.Function;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.algaeendeffector.AlgaeEndEffector;
import frc.robot.subsystems.arm.ArmJoint;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intakeextender.IntakeExtender;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.ReefPositionsUtil;
import frc.robot.util.ReefPositionsUtil.DeAlgaeLevel;
import frc.robot.util.ReefPositionsUtil.ScoreLevel;

public class ReefScoreCommandFactory {

    private static ReefScoreCommandFactory instance;

    public static enum ReefPosition {
        Left,
        Center,
        Right,
    }
    private static Alliance alliance;
    private final static int[] targetIdsRed = {
        6,7,8,9,10,11
    };

    private final static int[] targetIdsBlue = {
        17,18,19,20,21,22
    };

    private static int[] targetIds;

    //#region TODO Find Accurate Values
    private static LoggedTunableNumber offsetBBackingUp = new LoggedTunableNumber("AutoAlign/offsetBBackingUp", 1.0);
    private static LoggedTunableNumber rightOffsetBFinal = new LoggedTunableNumber("AutoAlign/rightOffsetBFinal", 0.68);
    private static LoggedTunableNumber leftOffsetBFinal = new LoggedTunableNumber("AutoAlign/leftOffsetBFinal", 0.68);
    private static LoggedTunableNumber algaeOffsetBFinal = new LoggedTunableNumber("AutoAlign/algaeOffsetBFinal", 0.55);
    private static LoggedTunableNumber offsetL = new LoggedTunableNumber("AutoAlign/offsetL", 0.155 + Inches.of(1).in(Meters));
    private static LoggedTunableNumber offsetR = new LoggedTunableNumber("AutoAlign/offsetR", 0.2 - Inches.of(3).in(Meters));
    private static LoggedTunableNumber offsetCL = new LoggedTunableNumber("AutoAlign/offsetCL", Inches.of(6).in(Meters));
    //#endregion

    //Overrides
    private static LoggedTunableNumber offsetLL3 = new LoggedTunableNumber("AutoAlign/offsetLL3", 0.155 + Inches.of(1).in(Meters));
    private static LoggedTunableNumber offsetRL3 = new LoggedTunableNumber("AutoAlign/offsetRL3", 0.2 - Inches.of(3).in(Meters));
    private static LoggedTunableNumber offsetLL2 = new LoggedTunableNumber("AutoAlign/offsetLL2", 0.155 + Inches.of(1).in(Meters));
    private static LoggedTunableNumber offsetRL2 = new LoggedTunableNumber("AutoAlign/offsetRL2", 0.2 - Inches.of(3).in(Meters));
    private static LoggedTunableNumber offsetBFinalL2 = new LoggedTunableNumber("AutoAlign/offsetBFinalL2", 0.63);
    private static LoggedTunableNumber offsetBFinalL3 = new LoggedTunableNumber("AutoAlign/offsetBFinalL3", 0.63);

    /**
     * Finds the closest april tag to a position.
     * 
     * @param pos The Pose2d to find the closest relative tag.
     * @param targets The list of AprilTag IDs to check for.
     * @return The pose of the closest april tag in "targets" to "pos"
     */
    private static Pose2d findClosestPose(Pose2d pos) {
        int[] targets = targetIds;
        double minDistance = Double.MAX_VALUE;
        Pose2d target = Pose2d.kZero;
        
        for (int i = 0; i < targets.length; i++) {
            double distance = pos.getTranslation().getDistance(aprilTagLayout.getTagPose(targets[i]).orElse(Pose3d.kZero).getTranslation().toTranslation2d());
            if (distance < minDistance) {
                target = aprilTagLayout.getTagPose(targets[i]).orElse(Pose3d.kZero).toPose2d();
                minDistance = distance;
            }
        }
        
        return target;
    }

    /**
     * Returns A function which takes the current position on the robot and returns where we want to score.
     * @param pos
     * @param isBackingUp
     * @return
     */
    public static Function<Pose2d, Pose2d> getGetTargetPositionFunction(ReefPosition pos, boolean isBackingUp) {
        refreshAlliance();
        return (Pose2d pose) -> {
            double backOffset = leftOffsetBFinal.get();
            double appliedOffset = 0;

            double offsetRForLevel = offsetR.getAsDouble();
            double offsetLForLevel = offsetL.getAsDouble();
            double offsetCForLevel = offsetCL.getAsDouble();
            switch (pos) {
                case Right:
                    switch (ReefPositionsUtil.getInstance().getScoreLevel()) {
                        case L2:
                            offsetRForLevel = offsetRL2.getAsDouble();
                            offsetLForLevel = offsetLL2.getAsDouble();
                            backOffset = offsetBFinalL2.getAsDouble();
                            break;
                        case L3:
                            offsetRForLevel = offsetRL3.getAsDouble();
                            offsetLForLevel = offsetLL3.getAsDouble();
                            backOffset = offsetBFinalL3.getAsDouble();
                            break;
                        default:
                            backOffset = rightOffsetBFinal.get();
                            break;
                    }
                    appliedOffset = offsetRForLevel;
                    break;
                case Left:
                    switch (ReefPositionsUtil.getInstance().getScoreLevel()) {
                        case L2:
                            offsetRForLevel = offsetRL2.getAsDouble();
                            offsetLForLevel = offsetLL2.getAsDouble();
                            backOffset = offsetBFinalL2.getAsDouble();
                            break;
                        case L3:
                            offsetRForLevel = offsetRL3.getAsDouble();
                            offsetLForLevel = offsetLL3.getAsDouble();
                            backOffset = offsetBFinalL3.getAsDouble();
                            break;
                        default:
                            backOffset = leftOffsetBFinal.get();
                            break;
                    }
                    appliedOffset = -offsetLForLevel;
                    break;
                default:
                    appliedOffset = -offsetCForLevel;
                    backOffset = algaeOffsetBFinal.get();
                    break;
            }
            
            Transform2d offset = new Transform2d(isBackingUp ? offsetBBackingUp.getAsDouble() : backOffset, appliedOffset, Rotation2d.kZero);
            Pose2d closestTarget = findClosestPose(pose);

            Pose2d target = closestTarget.transformBy(offset);
            Logger.recordOutput("TargetPose",target);
            return target;
        };
    }

    public static void initialize() {
        refreshAlliance();
    }

    public static void refreshAlliance() {
        targetIds = DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Blue ? targetIdsBlue : targetIdsRed;
    }

    /**
     * Gets a command which aligns the robot to the nearest reef position
     * @param position Left or right
     * @param isBackingUp Whether or not we should back up if we're too close to the reef
     */
    public static Command getNewAlignToReefCommand(ReefPosition position, boolean isBackingUp, Drive drive) {
        Function<Pose2d, Pose2d> positionFunction = getGetTargetPositionFunction(position, isBackingUp);
        //Base command
        Command returnedCommand = new AutoAlignCommand(getGetTargetPositionFunction(position, isBackingUp), drive)
        .raceWith(new RunCommand(() -> Logger.recordOutput("AutoAlign/DistanceMeters",drive.getDistanceTo(positionFunction.apply(drive.getAutoAlignPose())).in(Meters))));
        //If we're backing up, add kill conditions
        if(isBackingUp) {
            returnedCommand = returnedCommand
                // Kill when we are out of the distance (not necessary since we kill)
                // .until(() -> (drive.getDistanceTo(positionFunction.apply(drive.getPose())).in(Meters) > offsetBBackingUp.getAsDouble()))
                // Don't run the backup if we are out of the distance
                .unless(() -> {
                    double dist = drive.getDistanceTo(positionFunction.apply(drive.getAutoAlignPose())).in(Meters);
                    Logger.recordOutput("AutoAlign/DistanceMeters",dist);
                    return dist > offsetBBackingUp.getAsDouble();
                });
        }
        return returnedCommand;
    }

    /**
     * Gets a command which aligns the robotto the nearest reef position and prepares to score the coal, backing up if it's too close
     * @param position
     * @param coralLevelCommands
     * @param scoreCoralLevelCommands
     * @param drive
     * @return
     */
    public static Command getNewReefCoralScoreSequence(ReefPosition position, boolean isBackingUp, Map<ReefPositionsUtil.ScoreLevel,Command> coralLevelCommands, Map<ReefPositionsUtil.ScoreLevel,Command> scoreCoralLevelCommands, Map<ReefPositionsUtil.ScoreLevel,Command> stopCoralLevelCommands, Drive drive) {
        return 
            getNewAlignToReefCommand(position, true, drive).onlyIf(()->isBackingUp)
                .andThen(DriveCommands.brakeDrive(drive))
                .alongWith(ReefPositionsUtil.getInstance().getCoralLevelSelector(coralLevelCommands))
            .andThen(getNewAlignToReefCommand(position, false, drive))
            .andThen(ReefPositionsUtil.getInstance().getCoralLevelSelector(scoreCoralLevelCommands))
            // Added wait for score L4 so no need for wait here
            .andThen(
                getNewAlignToReefCommand(position, true, drive).onlyIf(()->ReefPositionsUtil.getInstance().isSelected(ScoreLevel.L4))
                    .andThen(ReefPositionsUtil.getInstance().getCoralLevelSelector(stopCoralLevelCommands))
            );
    }

    public static Command getNewReefCoralScoreSequence(ReefPosition position, boolean isBackingUp, Drive drive) {
        return getNewAlignToReefCommand(position, true, drive).onlyIf(()->isBackingUp)
            .andThen(DriveCommands.brakeDrive(drive))
            .andThen(getNewAlignToReefCommand(position, false, drive));
    }

    /**
     * Gets a command that scores an algae in auto
     * @param drive
     * @param shoulder
     * @param elbow
     * @param elevator
     * @param wrist
     * @param coralEE
     * @param algaeEE
     * @return
     */
    public static Command getNewAlgaePluckAutoAlignSequenceCommand(
        DeAlgaeLevel level, 
        Drive drive,
        ArmJoint shoulder, 
        ArmJoint elbow, 
        Elevator elevator, 
        Wrist wrist, 
        AlgaeEndEffector algaeEE,
        IntakeExtender extender
    ) {
        return getNewAlignToReefCommand(ReefPosition.Center, true, drive)
            .andThen(new ConditionalCommand(
                new TakeAlgaeL2(shoulder, elbow, wrist, algaeEE, elevator),
                new TakeAlgaeL3(shoulder, elbow, wrist, algaeEE, elevator),
                () -> ReefPositionsUtil.getInstance().isSelected(DeAlgaeLevel.Low)))
            .andThen(getNewAlignToReefCommand(ReefPosition.Center, false, drive))
            .until(algaeEE.hasAlgaeTrigger().debounce(0.5))
            .andThen(new WaitUntilCommand(algaeEE.hasAlgaeTrigger()))
            .andThen(getNewAlignToReefCommand(ReefPosition.Center, true, drive))
            .andThen(new AlgaeStowCommand(shoulder, elbow, elevator, wrist, algaeEE, extender));
    }
    
    public static Command getNewAlgaePluckAutoAlignCommand(
        Drive drive,
        boolean isBackingUp
    ) {
        return getNewAlignToReefCommand(ReefPosition.Center, isBackingUp, drive);
    }
}
