package frc.robot.commands;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.Constants;
import frc.robot.subsystems.feeder;
import frc.robot.subsystems.intake;
import frc.robot.subsystems.launcher;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.commands.DriveCommands;

public class ComplexCommands {

    public static feeder m_feeder = new feeder();
    public static intake m_intake = new intake();
    private static launcher m_launcher = new launcher();
    XboxController driverXbox = new XboxController(0);
  
    public static final Drive m_drive = new Drive(new GyroIOPigeon2(),
            new ModuleIOTalonFX(0),
            new ModuleIOTalonFX(1),
            new ModuleIOTalonFX(2),
            new ModuleIOTalonFX(3));

    private ComplexCommands(){}

    public static Command intakeCommand(XboxController controller){
        return Commands.sequence(
            m_feeder.feederIntakeSequence().alongWith(m_intake.intakeGo(-0.9)),
            m_intake.intakeGo(0)
        ).withTimeout(2).andThen(stopFeeder());
        
    }

    public static Command stopIntake(){
        return Commands.runOnce(() -> {
            m_intake.intakeGo(0);
        });
    }
    public static Command stopFeeder(){
        return Commands.runOnce(() -> {
            m_feeder.feederGo(0);
        });
    }


    /*public static Command launchCommand(){
        return Commands.sequence(
            m_launcher.launcherUptoSpeed(Constants.LauncherConstants.launcherSpeed),
            //broken    m_feeder.feederGo(0.1).withTimeout(0.5),
            m_launcher.launcherSetVelocity(0, 0)//,
            //broekn    m_feeder.feederGo(0)
        );
    }*/
    public static Command simpleLaunch(){
        return Commands.sequence(
        Commands.runOnce(() -> {m_launcher.launcherGo(0.9);}),
        Commands.waitSeconds(1),
        Commands.runOnce(() -> {m_feeder.feederGo(0.5);}),
        Commands.waitSeconds(1),
        Commands.runOnce(() -> {m_feeder.feederGo(0);}),
        Commands.runOnce(() -> {m_launcher.launcherGo(0);})
        );
    }

    public static Command shuttleLaunch(){
        return Commands.sequence(
        Commands.runOnce(() -> {m_launcher.launcherGo(0.25);}),
        Commands.waitSeconds(1),
        Commands.runOnce(() -> {m_feeder.feederGo(0.5);}),
        Commands.waitSeconds(1),
        Commands.runOnce(() -> {m_feeder.feederGo(0);}),
        Commands.runOnce(() -> {m_launcher.launcherGo(0);})
        );
    }
    public static Command driveAndShoot(){
        return Commands.sequence(
            
           DriveCommands.manualDrive(m_drive,-0.45,0.0,0.0).withTimeout(3.0),
           DriveCommands.manualDrive(m_drive,0.0,0.0,0.0).withTimeout(0.01),
           
           //Commands.runOnce(() -> {m_launcher.launcherGo(0.9);})
            simpleLaunch()
        );
    }
    public static Command aimRightDriveAndShoot(){
        return Commands.sequence(
            
           DriveCommands.manualDrive(m_drive,-0.75,0.0,0.2).withTimeout(5.0),
           DriveCommands.manualDrive(m_drive,0.0,0.0,0.0).withTimeout(0.01),
           
           //Commands.runOnce(() -> {m_launcher.launcherGo(0.9);})
            simpleLaunch()
        );
    }
    public static Command aimLeftDriveAndShoot(){
        return Commands.sequence(
            
           DriveCommands.manualDrive(m_drive,-0.65,0.0,-0.2).withTimeout(3.0),
           DriveCommands.manualDrive(m_drive,0.0,0.0,0.0).withTimeout(0.01),
           
           //Commands.runOnce(() -> {m_launcher.launcherGo(0.9);})
            simpleLaunch()
        );
    }
}
