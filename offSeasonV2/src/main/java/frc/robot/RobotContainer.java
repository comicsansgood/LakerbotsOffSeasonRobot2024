// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.LauncherConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ComplexCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.feeder;
import frc.robot.subsystems.intake;
import frc.robot.subsystems.launcher;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIOTalonFX;

import java.io.File;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ComplexCommands;


public class RobotContainer {

  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
 // private final feeder m_feeder = new feeder();
  //private final intake m_intake = new intake();
 // private final launcher m_launcher = new launcher();

  public static final Drive m_drive = new Drive(new GyroIOPigeon2(),
            new ModuleIOTalonFX(0),
            new ModuleIOTalonFX(1),
            new ModuleIOTalonFX(2),
            new ModuleIOTalonFX(3));

XboxController driverXbox = new XboxController(0);//Seperate controller object for the drivetrain controlling to manage code complexity

private final LoggedDashboardChooser<Command> m_chooser;


  public RobotContainer() {

    m_chooser = new LoggedDashboardChooser<>("auto choices", AutoBuilder.buildAutoChooser());
    NamedCommands.registerCommand("simple launch", ComplexCommands.simpleLaunch());

    configureBindings();

    m_chooser.addDefaultOption("null", Commands.none());
    //m_chooser.addOption("Blue Side",new ComplexCommands.)

     m_drive.setDefaultCommand(
          DriveCommands.joystickDrive(
              m_drive,
              () -> -driverXbox.getLeftY(),
              () -> -driverXbox.getLeftX(),
              () -> driverXbox.getRightX(), 
              () -> driverXbox.getLeftTriggerAxis())
      );

      ComplexCommands.m_intake.setDefaultCommand(ComplexCommands.m_intake.intakeGo(0.0));

  }


  private void configureBindings() {
  
    //intake command    
    new JoystickButton(driverXbox, XboxController.Button.kRightBumper.value).onTrue(ComplexCommands.intakeCommand(driverXbox));

    //launch command
    new JoystickButton(driverXbox, XboxController.Button.kLeftBumper.value).onTrue(ComplexCommands.simpleLaunch());

    //shuttle
    new JoystickButton(driverXbox, XboxController.Button.kX.value).onTrue(ComplexCommands.shuttleLaunch());

    new JoystickButton(driverXbox, XboxController.Button.kA.value).onTrue(DriveCommands.manualDrive(m_drive,0.2,0.0,0.0));

    new JoystickButton(driverXbox, XboxController.Button.kStart.value).onTrue(DriveCommands.zeroGyro(m_drive));

    new JoystickButton(driverXbox, XboxController.Button.kY.value).onTrue(ComplexCommands.driveAndShoot());

    //new JoystickButton(driverXbox, XboxController.Button.kB.value).onTrue(ComplexCommands.stopIntake());
  }


  public Command getAutonomousCommand() {
    return m_chooser.get();  
    //return ComplexCommands.driveAndShoot();
    //return ComplexCommands.aimRightDriveAndShoot();
    //return ComplexCommands.aimLeftDriveAndShoot();
  } 
}
