// package frc;

// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.RelativeEncoder;
// import edu.wpi.first.math.controller.ElevatorFeedforward;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// public class ElevadorTESTE extends SubsystemBase {
//     // Definição das portas CAN do motor
//     private static final int MOTOR_ID = 11;

//     // Constantes do perfil de movimento e controle PID
//     private static final double kDt = 0.02; // Tempo de ciclo (20ms)
//     private static final double kMaxVelocity = 1.75; // Velocidade máxima do elevador
//     private static final double kMaxAcceleration = 0.75; // Aceleração máxima
//     private static final double kP = 1.3;
//     private static final double kI = 0.0;
//     private static final double kD = 0.7;
//     private static final double kS = 1.1;
//     private static final double kG = 1.2;
//     private static final double kV = 1.3;

//     // Definição dos níveis do elevador (em metros)
//     public static final double LEVEL_1 = 0.46;
//     public static final double LEVEL_2 = 0.81;
//     public static final double LEVEL_3 = 1.22;
//     public static final double LEVEL_4 = 1.84;

//     // Hardware
//     private final SparkMax motor;
//     private final RelativeEncoder encoder;
//     private final ProfiledPIDController controller;
//     private final ElevatorFeedforward feedforward;

//     public ElevadorTESTE() {
//         motor = new SparkMax(MOTOR_ID, MotorType.kBrushless);
//         encoder = motor.getEncoder();

//         controller = new ProfiledPIDController(kP, kI, kD, new TrapezoidProfile.Constraints(kMaxVelocity, kMaxAcceleration), kDt);
//         feedforward = new ElevatorFeedforward(kS, kG, kV);
//     }

//     /**
//      * Define a altura do elevador para um nível desejado
//      */
//     public void setHeight(double height) {
//         controller.setGoal(height);
//     }

//     /**
//      * Método que é executado periodicamente para controlar o elevador
//      */
//     @Override
//     public void periodic() {
//         double output = controller.calculate(encoder.getPosition()) + feedforward.calculate(controller.getSetpoint().velocity);
//         motor.setVoltage(output);

//         // Atualiza os valores no SmartDashboard para diagnóstico
//         SmartDashboard.putNumber("Elevador - Altura Atual", encoder.getPosition());
//         SmartDashboard.putNumber("Elevador - Saída do Motor", output);
//     }
// }
