// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.Controle;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
  // Aqui iniciamos o swerve

  private final Elevator myElevator = new Elevator();
  

  // private final Joystick m_joystick = new Joystick(1);
  


  private SwerveSubsystem swerve = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));

  // É criado o escolhedor de autônomo
  private final SendableChooser<Command> autoChooser;

  // Controle de Xbox, troque para o qual sua equipe estará utilizando
  private CommandXboxController controleXbox = new CommandXboxController(Controle.xboxControle);
  private CommandXboxController xboxElevador = new CommandXboxController(Controle.xboxAlga);

  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(swerve.getSwerveDrive(),
      () -> controleXbox.getLeftY() * 1,
      () -> controleXbox.getLeftX() * 1)
      .withControllerRotationAxis(() -> controleXbox.getRightX() * -1)
      .deadband(Constants.Controle.DEADBAND)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);

  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
      .withControllerHeadingAxis(() -> controleXbox.getRightX() * 1,
      () -> controleXbox.getRightY() * -1)
      .headingWhile(true);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(swerve.getSwerveDrive(),
      () -> -controleXbox.getLeftY(),
      () -> -controleXbox.getLeftX())
      .withControllerRotationAxis(() -> controleXbox.getRawAxis(
          2))
      .deadband(Constants.Controle.DEADBAND)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);

  SwerveInputStream driveDirectAngleKeyboard = driveAngularVelocityKeyboard.copy()
      .withControllerHeadingAxis(() -> Math.sin(
          controleXbox.getRawAxis(
              2) *
              Math.PI)
          *
          (Math.PI *
              2),
          () -> Math.cos(
              controleXbox.getRawAxis(
                  2) *
                  Math.PI)
              *
              (Math.PI *
                  2))
      .headingWhile(true);

  public RobotContainer() {

    NamedCommands.registerCommand("Intake", new PrintCommand("Intake"));

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    // Configure the trigger bindings
    configureBindings();
  }

  // Função onde os eventos (triggers) são configurados
  private void configureBindings() {

    Command driveFieldOrientedAngularVelocity = swerve.driveFieldOriented(driveAngularVelocity);
    swerve.setDefaultCommand(driveFieldOrientedAngularVelocity);

    controleXbox.a().onTrue(Commands.runOnce(swerve::zeroGyro, swerve));

    controleXbox.rightBumper().whileTrue(swerve.alignInDegress(
        () -> MathUtil.applyDeadband(controleXbox.getLeftY(), Constants.Controle.DEADBAND),
        () -> MathUtil.applyDeadband(controleXbox.getLeftX(), Constants.Controle.DEADBAND),
        45));

    controleXbox.leftBumper().whileTrue(swerve.alignInDegress(
        () -> MathUtil.applyDeadband(controleXbox.getLeftY(), Constants.Controle.DEADBAND),
        () -> MathUtil.applyDeadband(controleXbox.getLeftX(), Constants.Controle.DEADBAND),
        -45));

    if (!Robot.isReal()) {
      controleXbox.start().onTrue(Commands.runOnce(() -> swerve.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
    }



    xboxElevador.b().whileTrue( myElevator.ElevatorPos1());  //define as posições do elevador
  
    xboxElevador.x().whileTrue( myElevator.ElevatorPos2());

    xboxElevador.y().whileTrue( myElevator.ElevatorPos3());

    xboxElevador.a().whileTrue( myElevator.ElevatorPos4());

}

  

  // Função que retorna o autônomo
  public Command getAutonomousCommand() {
    // Aqui retornamos o comando que está no selecionador
    return autoChooser.getSelected();
  }

  // Define os motores como coast ou brake
  public void setMotorBrake(boolean brake) {
    swerve.setMotorBrake(brake);
  }

  public void setHeadingCorrection(boolean heading){
    swerve.setHeadingCorrection(heading);
  }
}
