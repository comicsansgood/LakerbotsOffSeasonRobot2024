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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;


public class RobotContainer {

  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
 // private final feeder m_feeder = new feeder();
 // private final intake m_intake = new intake();
 // private final launcher m_launcher = new launcher();

  public static final Drive m_drive = new Drive(new GyroIOPigeon2(),
            new ModuleIOTalonFX(0),
            new ModuleIOTalonFX(1),
            new ModuleIOTalonFX(2),
            new ModuleIOTalonFX(3));

XboxController driverXbox = new XboxController(0);//Seperate controller object for the drivetrain controlling to manage code complexity
private final CommandXboxController controller = new CommandXboxController(0);



  public RobotContainer() {
    configureBindings();

     m_drive.setDefaultCommand(
          DriveCommands.joystickDrive(
              m_drive,
              () -> -driverXbox.getLeftY(),
              () -> -driverXbox.getLeftX(),
              () -> -driverXbox.getRightX(), 
              () -> driverXbox.getLeftTriggerAxis()));

  }


  private void configureBindings() {
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

     
    
        //intake command
    //controller.rightBumper().onTrue(ComplexCommands.intakeCommand());

    //launch command
    //controller.leftBumper().onTrue(ComplexCommands.launchCommand());    

    
  }


  public Command getAutonomousCommand() {
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
