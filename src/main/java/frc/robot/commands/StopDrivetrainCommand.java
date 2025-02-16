package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

public class StopDrivetrainCommand extends Command {
    private Drive drive;
    
        public StopDrivetrainCommand(Drive drivetrain) {
            drive = drivetrain;
    }

    @Override
        public boolean isFinished() {
            return true;
        }

    @Override
    public void initialize() {
        drive.stop();
    }
}