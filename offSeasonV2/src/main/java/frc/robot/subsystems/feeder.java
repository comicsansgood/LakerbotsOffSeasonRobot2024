
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.ComplexCommands;

public class feeder extends SubsystemBase {

  public CANSparkMax feederMotor = new CANSparkMax(12, MotorType.kBrushless);
  public final DigitalInput noteSensor = new DigitalInput(1);


  public feeder() {
    feederMotor.restoreFactoryDefaults();
    feederMotor.setCANTimeout(250);
    feederMotor.setInverted(false);
    feederMotor.enableVoltageCompensation(12.0);
    feederMotor.setSmartCurrentLimit(30);
    feederMotor.burnFlash();
   
    
  }

  public void feederGo(double speed) {
    feederMotor.set(speed);
  }

  public boolean isNoteDetected(){
    return !noteSensor.get();
  }

  public Command feederGoUntilNoteDetected(){
    return runOnce(()-> {
      feederGo(.3);}).andThen(Commands.waitUntil(() -> isNoteDetected()));
  }

  public Command feederGoUntilNoteNotDetected(){
    return runOnce(()-> {
      feederGo(.3);}).andThen(Commands.waitUntil(() -> !isNoteDetected()));
  };


  public Command feederIntakeSequence(){
  
    return Commands.sequence(
      feederGoUntilNoteDetected(),
      runOnce(() -> {feederGo(0);}),
      Commands.waitSeconds(0.2),
      feederGoUntilNoteNotDetected(),
      runOnce(() -> {feederGo(0);}),
      Commands.waitSeconds(0.2),
      runOnce(() -> {feederGo(-0.1);}),
      Commands.waitSeconds(0.5),
      runOnce(() -> {feederGo(0);})


      //runOnce(() -> {System.out.println("DONE\nDONE\nDONE\nDONE\nDONE");})//, 
      /*runOnce(() -> {feederGo(0);}), 
      feederGoUntilNoteNotDetected(), 
      runOnce(() -> {feederGo(-0.1);}), 
      Commands.waitSeconds(0.5), 
      runOnce(() -> {feederGo(0);})*/
    );
  }

  public Command exampleMethodCommand() {

    return runOnce(
        () -> {
        });
  }

  
 
  

  @Override
  public void periodic() {
        SmartDashboard.putBoolean("Note detected", isNoteDetected());

  }

  @Override
  public void simulationPeriodic() {
  }
}
