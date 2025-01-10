// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
//import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.R2Jesu_Limelight;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.studica.frc.AHRS;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.REVLibError;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DigitalInput;

public class R2Jesu_ShooterSubsystem extends SubsystemBase {
  /** Creates a new R2Jesu_ShooterSubsystem. */
  public R2Jesu_ShooterSubsystem(R2Jesu_Limelight x) {
    this.shooterLimelight=x;
  }
  private SparkMax topMotor = new SparkMax(11, MotorType.kBrushless);
  private RelativeEncoder topMotorEncoder = topMotor.getEncoder();
  private SparkMax bottomMotor = new SparkMax(10, MotorType.kBrushless);
  private RelativeEncoder bottomMotorEncoder = bottomMotor.getEncoder();
  private SparkMax indexerMotor = new SparkMax(9, MotorType.kBrushless);
  private RelativeEncoder indexerMotorEncoder = indexerMotor.getEncoder();
  private SparkMax intakeMotor = new SparkMax(12, MotorType.kBrushless);
  private SparkMax intakeGreenMotor = new SparkMax(13, MotorType.kBrushless);
  private DigitalInput digitalSensor = new DigitalInput(0);
  private R2Jesu_Limelight shooterLimelight;
  private double shootIt, theDist, lastDist;
  private boolean shooterRunning;


  /**
   * R2Jesu_Shooter command factory method.
   *
   * @return a command
   */
  public Command R2Jesu_ShooterMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An R2Jesu_Shooter method querying a boolean state of the subsystem (for R2Jesu_Shooter, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean R2Jesu_ShooterCondition() {
    // Query some boolean state, such as a digital sensor.
    return true;
  }

  public void runShooter() {
    //System.out.println("runShooter");
    if (!digitalSensor.get()) {
      intakeMotor.set(0.30);
      intakeGreenMotor.set(.30);
      indexerMotor.set(0.10);
      topMotor.set(0.0);
      bottomMotor.set(0.0);
      shooterRunning = false;
  }
  else {
      theDist=shooterLimelight.getDistance();
      if (Math.abs(theDist-lastDist) > 2.0 || shooterRunning == false) {
          if (theDist == 0.0) {
              shootIt =.95;
          }
          else if (theDist <= 60.0) {
              shootIt =.90;
          }
          else if (theDist <= 80.0) {
              shootIt = .60;
          }
          else {
              shootIt = .50;
          }
          //shootIt=SmartDashboard.getNumber("Set It", 0.0);//Take out ater for variation
          //shootIt=.575;//Take out ater for variation
          topMotor.set(shootIt);//.95
          bottomMotor.set(shootIt);//.9
          intakeMotor.set(0.0);
          intakeGreenMotor.set(0.0);
          indexerMotor.set(0.0);

          shooterRunning = true;
          lastDist=theDist;
      }
  }
}  
public void standardShot() {
  //System.out.println("standardShot");
  indexerMotor.set(0.15);
    
  }

  public void lightShot() {
    //System.out.println("lightShot");
    topMotor.set(0.25);
    bottomMotor.set(0.25);
    intakeMotor.set(0.0);
    intakeGreenMotor.set(0.0);
    indexerMotor.set(0.10);
}

public void reverseIntake() {
//System.out.println("reverseIntake");
topMotor.set(0.0);
bottomMotor.set(0.0);
intakeMotor.set(-0.20);
intakeGreenMotor.set(-0.20);
indexerMotor.set(0.0);
}
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
