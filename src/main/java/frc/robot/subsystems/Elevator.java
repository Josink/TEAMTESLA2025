package frc.robot.subsystems;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
    private final TalonFX leftElevatorMotor = new TalonFX(57);
    private final TalonFX follower = new TalonFX(58);
    private final DutyCycleOut duty = new DutyCycleOut(0);
    private final TalonFXConfiguration talonConfigs;
    private final SoftwareLimitSwitchConfigs sl;
    private final FeedbackConfigs fb;

    public Elevator(){
        talonConfigs = new TalonFXConfiguration();
        fb = talonConfigs.Feedback;
        sl = talonConfigs.SoftwareLimitSwitch;

        setElevatorPosition(-0.25);
        applyElevatorMotorConfigs(InvertedValue.Clockwise_Positive);
        follower.setControl(new Follower(57, true));
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Elevator Encoder: ", getElevatorPosition());
        SmartDashboard.putString("A BUTTON", "STOW");
        SmartDashboard.putString("B BUTTON", "SAFE_TO_MOVE");
        SmartDashboard.putString("X BUTTON", "L_1");
        SmartDashboard.putString("B BUTTON", "GROUND_INTAKE");
        SmartDashboard.putBoolean("STOP?", stoppingornot());
    }
    
    public void setElevatorMotorSpeed(double speed){
        duty.Output = speed;
        duty.EnableFOC = true;
        leftElevatorMotor.setControl(duty);
    }

    public void noStops(){
        sl.ForwardSoftLimitEnable = false;
    }

    public void newStop(){
        sl.ForwardSoftLimitEnable = true;
        resetElevatorPosition();
    }

    public boolean stoppingornot(){
        return sl.ForwardSoftLimitEnable;
    }

    public Command elevateManually(double speed){
        duty.Output = speed;
        duty.EnableFOC = true;
        return run(
            ()->{
                leftElevatorMotor.setControl(duty);
            }
        );
    }

    public void stopElevatorMotors(){
        leftElevatorMotor.set(0);
    }

    public double getElevatorPosition(){
        return leftElevatorMotor.getPosition().getValueAsDouble();
    }

    public void setElevatorPosition(double pos){
        leftElevatorMotor.setPosition(pos);
    }

    public void resetElevatorPosition(){
        leftElevatorMotor.setPosition(0);
    }

    public void GoToPos(double pos){
        final MotionMagicTorqueCurrentFOC request = new MotionMagicTorqueCurrentFOC(pos);
        leftElevatorMotor.setControl(request);
    }

    private void applyElevatorMotorConfigs(InvertedValue inversion){
        fb.SensorToMechanismRatio = 15;

        sl.ForwardSoftLimitEnable = true;
        sl.ForwardSoftLimitThreshold = 0;
        sl.ReverseSoftLimitEnable = true;
        sl.ReverseSoftLimitThreshold = -5.9;

        talonConfigs.Slot0.kP = 27;
        talonConfigs.Slot0.kI = 7;
        talonConfigs.Slot0.kD = 0;
        talonConfigs.Slot0.kV = 0;
        talonConfigs.Slot0.kG = 0.29;
        talonConfigs.Slot0.kS = 0;
        talonConfigs.Slot0.GravityType = GravityTypeValue.Elevator_Static;

        var motionMagicConfigs = talonConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 50;
        motionMagicConfigs.MotionMagicAcceleration = 100;
        motionMagicConfigs.MotionMagicJerk = 1000;

        talonConfigs.Feedback.FeedbackRemoteSensorID = leftElevatorMotor.getDeviceID();
        talonConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        
        leftElevatorMotor.getConfigurator().apply(talonConfigs);
        follower.getConfigurator().apply(talonConfigs);

        MotorOutputConfigs motorOutputConfigsRight = new MotorOutputConfigs();
        motorOutputConfigsRight.NeutralMode = NeutralModeValue.Brake;
        MotorOutputConfigs motorOutputConfigsLeft = new MotorOutputConfigs();
        motorOutputConfigsLeft.NeutralMode = NeutralModeValue.Brake;
        follower.getConfigurator().apply(motorOutputConfigsRight);
        leftElevatorMotor.getConfigurator().apply(motorOutputConfigsLeft);
  }
}
