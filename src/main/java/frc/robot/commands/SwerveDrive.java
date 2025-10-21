//moving over to other git account
//trying again
package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Vision.Limelight;
import frc.robot.Vision.LimelightHelpers;
import frc.robot.commands.AprilTagPositions.LockInReef;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;

public class SwerveDrive extends Command {

    private CommandSwerveDrivetrain swerve;
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    
    private SendableChooser<Double> m_speedChooser;

    private SwerveRequest m_Request;

    // Use open-loop control for drive motors
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage); 
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    private CommandXboxController driverController;

    private double rotationVal, xVal, yVal;
    private SlewRateLimiter slewR, slewX, slewY;

    private Elevator elevator;
    private LockInReef lockInReef;
    private Limelight ll;

    public SwerveDrive(CommandSwerveDrivetrain swerve, CommandXboxController driver, Elevator elevator){
        this.swerve = swerve;
        this.driverController = driver;
        this.elevator = elevator;
        addRequirements(swerve);

        slewR = new SlewRateLimiter(MaxSpeed*0.85);
        slewX = new SlewRateLimiter(MaxSpeed*0.85);
        slewY = new SlewRateLimiter(MaxSpeed*0.85);

        lockInReef = new LockInReef(swerve);
        ll = new Limelight();

        m_speedChooser = new SendableChooser<Double>();
        m_speedChooser.addOption("100%", 1.0);
        m_speedChooser.addOption("90%", 0.9);
        m_speedChooser.setDefaultOption("85%", 0.85);
        m_speedChooser.addOption("80%", 0.8);
        m_speedChooser.addOption("70%", 0.7);
        m_speedChooser.addOption("60%", 0.6);
        m_speedChooser.addOption("50%", 0.5);
        m_speedChooser.addOption("40%", 0.4);
        m_speedChooser.addOption("30%", 0.3);
        m_speedChooser.addOption("20%", 0.2);
        SmartDashboard.putData("Speed Percent", m_speedChooser);

    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        //change speed depending on how tall the elevator is to avoid tipping
        MaxSpeed = (1.3 * elevator.getElevatorPosition() + 9.46);
        
        xVal = MathUtil.applyDeadband(-driverController.getLeftX() * m_speedChooser.getSelected(),0.2);
        yVal = MathUtil.applyDeadband(-driverController.getLeftY() * m_speedChooser.getSelected(), 0.2);
        rotationVal = MathUtil.applyDeadband(-driverController.getRightX() * m_speedChooser.getSelected(), 0.1);

        driverController.rightBumper().whileTrue(swerve.applyRequest(() -> brake));
        driverController.leftBumper().onTrue(swerve.runOnce(() -> swerve.seedFieldCentric()));
        
        m_Request = drive.withVelocityX(slewY.calculate(yVal * MaxSpeed))
        .withVelocityY(slewX.calculate(xVal * MaxSpeed))
        .withRotationalRate(slewR.calculate(rotationVal * MaxAngularRate));

        swerve.setControl(m_Request);

        //april tag detection
        double aprilTagID = LimelightHelpers.getFiducialID("limelight-left");

        if (ll.blueSideReef((int) aprilTagID)){
            driverController.a().onTrue(lockInReef);
        }
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
