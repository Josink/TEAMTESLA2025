// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  private final TalonFX intakeMotor = new TalonFX(53);
  private final TalonFX intakeMotor2 = new TalonFX(62);
  private final TalonFX rotateMotor = new TalonFX(60);

  private DutyCycleOut intake = new DutyCycleOut(0);
  private DutyCycleOut intake2 = new DutyCycleOut(0);
  private DutyCycleOut rotate = new DutyCycleOut(0);

  public Arm() {
    setRotationPosition(1.065);
    applyRotateMotorConfigs(InvertedValue.Clockwise_Positive);
  }

  @Override
  public void periodic() {
    //This method will be called once per scheduler run
    SmartDashboard.putNumber("Rotate pos", getRotatePosition());
  }

  public void setIntakeSpeed(double ispeed){
    intake.Output = ispeed;
    intakeMotor.setControl(intake);
    intakeMotor2.setControl(intake);
  }

  public void manualControl(double rspeed, BooleanSupplier left, BooleanSupplier right, double ispeed, double ispeed2){
    rotate.Output = rspeed;
    rotate.EnableFOC = true;
    intake.EnableFOC = true;
    intake2.EnableFOC = true; 
    rotateMotor.setControl(rotate);
    if (left.getAsBoolean()){
      intake.Output = ispeed;
      intakeMotor.setControl(intake);
      intake2.Output = -ispeed;
      intakeMotor2.setControl(intake2);
    } else if (right.getAsBoolean()){
        intake.Output = -ispeed2;
        intakeMotor.setControl(intake);
        intake2.Output = ispeed2;
        intakeMotor2.setControl(intake2);
    } else{
      intakeMotor.set(0);
      intakeMotor2.set(0);
    }
  }

  public void stopIntakeMotor(){
    intakeMotor.set(0);
    intakeMotor2.set(0);
  }

  public void stopRotateMotor(){
    rotateMotor.set(0);
  }

  public double getRotatePosition(){
    return rotateMotor.getPosition().getValueAsDouble();
  }

  public void resetRotatePosition(){
    rotateMotor.setPosition(0);
  }

  public void setRotationPosition(double rPosition){
    rotateMotor.setPosition(rPosition);
  }

  public void rotateToPos(double pos){
    final MotionMagicTorqueCurrentFOC request = new MotionMagicTorqueCurrentFOC(pos);
    rotateMotor.setControl(request);
  }

  private void applyRotateMotorConfigs(InvertedValue inversion){
    TalonFXConfiguration talonConfigs = new TalonFXConfiguration();
    
    FeedbackConfigs fb = talonConfigs.Feedback;
    fb.SensorToMechanismRatio = 15;

    SoftwareLimitSwitchConfigs rw = talonConfigs.SoftwareLimitSwitch;
    rw.ForwardSoftLimitEnable = false;
    rw.ForwardSoftLimitThreshold = 0;
    rw.ReverseSoftLimitEnable = false;
    rw.ReverseSoftLimitThreshold = -7.6;

    talonConfigs.Slot0.kP = 15;
    talonConfigs.Slot0.kI = 3;
    talonConfigs.Slot0.kD = 0;
    talonConfigs.Slot0.kV = 0;
    talonConfigs.Slot0.kG = 0.29;
    talonConfigs.Slot0.kS = 0;
    talonConfigs.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

    var motionMagicConfigs = talonConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 50;
    motionMagicConfigs.MotionMagicAcceleration = 100;
    motionMagicConfigs.MotionMagicJerk = 1000;

    talonConfigs.Feedback.FeedbackRemoteSensorID = rotateMotor.getDeviceID();
    talonConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

    rotateMotor.getConfigurator().apply(talonConfigs);

    MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
    motorOutputConfigs.Inverted = inversion;
    motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
    rotateMotor.getConfigurator().apply(motorOutputConfigs);
  }

}
