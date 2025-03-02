package frc.robot.commands;

import static frc.robot.subsystems.vision.VisionConstants.aprilTagLayout;

import java.nio.file.Path;
import java.util.ArrayList;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryGenerator.ControlVectorList;
import frc.robot.subsystems.vision.VisionConstants;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.util.LoggedTunableNumber;

public class RoughAlignToReef extends Command {

    private static int[] targetIdsRed = {
        6,7,8,9,10,11
    };

    private static int[] targetIdsBlue = {
        17,18,19,20,21,22
    };

    private static int[] targetIds;

    private LoggedTunableNumber offsetB = new LoggedTunableNumber("AutoAlign/offsetB", 0.5);
    private LoggedTunableNumber offsetR = new LoggedTunableNumber("AutoAlign/offsetR", 0);
    private LoggedTunableNumber toleranceB = new LoggedTunableNumber("AutoAlign/toleranceB", 0.1);
    private LoggedTunableNumber toleranceR = new LoggedTunableNumber("AutoAlign/toleranceR", 0.1);
    private boolean leftSide = false;

    private Drive drivetrain;
    private AprilTagFieldLayout fieldTags;

    private Pose2d drivingPose = Pose2d.kZero;

    private double m_strafe;
    private double m_throttle;
    private double m_spin;
    private double m_tx;
    private double m_ty;
    private double m_tr;
    private DoubleSupplier spinSupplier;
    private PIDController strafePID = new PIDController(1.5, 0.0, 0.0);
    private PIDController distancePID = new PIDController(1.5, 0.0, 0.0);
    private PIDController spinPID = new PIDController(5.0, 0.0, 0.0);

    private final double MAX_STRAFE = 2; 
    private final double MAX_THROTTLE = 4;
    private static final double MAX_SPIN = Math.toRadians(180.0);
            
    /**
     * Returns the pose of the closest april tag in "targets" to "pos"
     * 
     * @param pos
     * @param targets
     * @return
     */
    private static Pose2d findClosestPose(Pose2d pos, int[] targets) {
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

    public RoughAlignToReef(Drive drivetrain, boolean leftSide, DoubleSupplier controllerRotationInput) {
        this.drivetrain = drivetrain;
        this.fieldTags = VisionConstants.aprilTagLayout;
        this.leftSide = leftSide;
        this.spinSupplier = controllerRotationInput;
    }

    private void resetTargetPose() {
        double localoffsetR = leftSide ? offsetR.getAsDouble() * -1 : offsetR.getAsDouble();
        Transform2d offset = new Transform2d(offsetB.getAsDouble(), localoffsetR, Rotation2d.kZero);
        Pose2d closestPose = findClosestPose(drivetrain.getPose(), targetIds);

        Pose2d targetPose = closestPose.transformBy(offset);
        drivingPose = targetPose;
        Logger.recordOutput("ClosestPose",targetPose);
    }

    private Pose2d getCurrentPose() {
        return drivetrain.getPose();
    }

    @Override
    public void initialize() {
        targetIds = DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red ? targetIdsRed : targetIdsBlue ;

        resetTargetPose();
        strafePID.reset();
        distancePID.reset();
        spinPID.reset();
    }
    /* 
    * This command utilitzes the swerve drive while it isn't field relative. 
    * The swerve drive returns back to field relative after the command is used.
    */
    @Override
    public void execute() {
        Pose2d RobotRelativeTargetPose = drivingPose.relativeTo(getCurrentPose());

        m_tx = -RobotRelativeTargetPose.getY();
        m_ty = -RobotRelativeTargetPose.getX();
        m_tr = RobotRelativeTargetPose.getRotation().unaryMinus().getRadians();

        m_strafe = MathUtil.clamp(strafePID.calculate(m_tx, 0.0), -MAX_STRAFE, MAX_STRAFE); 
        m_throttle = MathUtil.clamp(distancePID.calculate(m_ty, 0.0),-MAX_THROTTLE,MAX_THROTTLE);
        m_spin = MathUtil.clamp(spinPID.calculate(m_tr, 0.0),-MAX_SPIN,MAX_SPIN);
        // m_spin = spinSupplier.getAsDouble();


        ChassisSpeeds speeds =
        new ChassisSpeeds(
            m_throttle,
            m_strafe,
            m_spin);
        drivetrain.runVelocity(speeds);

        Logger.recordOutput("AlignTx/TX", m_tx);
        Logger.recordOutput("AlignTx/TZ", m_ty);
        Logger.recordOutput("AlignTx/TR", m_tr);
        Logger.recordOutput("AlignTx/strafe", m_strafe);
        Logger.recordOutput("AlignTx/throttle", m_throttle);
        Logger.recordOutput("AlignTx/spin", m_spin);
        Logger.recordOutput("AlignTx/TargetPose",drivingPose);
    }

    @Override
    public boolean isFinished() {
        return MathUtil.isNear(m_tx, 0.0,toleranceR.getAsDouble()) && MathUtil.isNear(m_ty, 0.0,toleranceB.getAsDouble());
    }
    
}
