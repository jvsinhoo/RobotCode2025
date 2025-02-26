package frc.robot;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import com.revrobotics.spark.SparkLowLevel.MotorType;

import java.util.logging.Level;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@SuppressWarnings("PMD.RedundantFieldInitializer")
public class Elevator extends SubsystemBase {
  private static double kDt = 0.02;  //Intervalo de tempo do loop (20ms).
  private static double kMaxVelocity = 1.75;  //Velocidade máxima do elevador.
  private static double kMaxAcceleration = 0.75;  //Aceleração máxima para suavizar movimentos.
  private static double kP = 1.3;
  private static double kI = 0.0;
  private static double kD = 0.7;
  private static double kS = 1.1;  // kS, kG, kV -> compensam efeitos físicos, como gravidade e atrito
  private static double kG = 1.2;
  private static double kV = 1.3;

  
 
//medidas em metros
  public class ElevatorConstants {
    public static final double level1 = 0.46;  
    public static final double level2 = 0.81;
    public static final double level3 = 1.22;
    public static final double level4 = 1.84;
}


// stage 1 46cm  -> 0.46 metros
// stage 2 81cm  -> 0.81 m...
// stage 3 122cm -> 1.22 m...
// stage 4 184cm -> 1.84 m...



  private final Encoder m_encoder = new Encoder(0, 1);  //Mede a altura do elevador (conectado nas portas digitais 1 e 2).
  private final SparkFlex m_motor = new SparkFlex(11, MotorType.kBrushless);  
  // private final PWMSparkMax m_motor = new PWMSparkMax(1);

  // Create a PID controller whose setpoint's change is subject to maximum
  // velocity and acceleration constraints.
  public final TrapezoidProfile.Constraints m_constraints =
      new TrapezoidProfile.Constraints(kMaxVelocity, kMaxAcceleration);  //Define as restrições de velocidade e aceleração.

  public final ProfiledPIDController m_controller =
      new ProfiledPIDController(kP, kI, kD, m_constraints, kDt);  // Controlador PID suavizado pelas restrições do perfil trapezoidal.

  private final ElevatorFeedforward m_feedforward = new ElevatorFeedforward(kS, kG, kV);  //Adiciona compensação para gravidade e atrito.

  public Elevator() {
    m_encoder.setDistancePerPulse(1.0 / 360.0 * 2.0 * Math.PI * 1.5);  
  }

  //   @Override
  // public void teleopPeriodic() {
  //   if (m_joystick.getRawButtonPressed(4)) {
  //     m_controller.setGoal(ElevatorConstants.level2);                      //Sobe o elevador até 5 metros.

  //   } else if (m_joystick.getRawButtonPressed(3)) {
  //     m_controller.setGoal(0);                      //Desce o elevador até 0 metros
  //   }

  //   // Run controller and update motor output
  //   m_motor.setVoltage(
  //       m_controller.calculate(m_encoder.getDistance())
  //           + m_feedforward.calculate(m_controller.getSetpoint().velocity));  //Aplica a potência ao motor para mover o elevador.
  // } 


  //posições do elevador 

  public void ElevatorReset () {

      m_encoder.reset ();

}

  public Command ElevatorPos1 () {

      
      // return run(() -> m_controller.setGoal(ElevatorConstants.level1));
     
        return run(() -> {
            m_controller.setGoal(ElevatorConstants.level1);
            double output = m_controller.calculate(m_encoder.getDistance()) + 
                            m_feedforward.calculate(m_controller.getSetpoint().velocity);
            m_motor.setVoltage(output);
        });
    
    
  }

public Command ElevatorPos2 () {

      
  return run(() -> m_controller.setGoal(ElevatorConstants.level2));

}

public Command ElevatorPos3 () {

      
  return run(() -> m_controller.setGoal(ElevatorConstants.level3));

}

public Command ElevatorPos4 () {

      
  return run(() -> m_controller.setGoal(ElevatorConstants.level4));

}


protected void execute(){

  SmartDashboard.putNumber("Valor Encoder", m_encoder.getDistance());
}
  }

 