// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.SwerveDrive;
import frc.robot.commands.AprilTagPositions.LockInReef;
import frc.robot.commands.ArmPositions.CoralL1;
import frc.robot.commands.ArmPositions.CoralL4;
import frc.robot.commands.ArmPositions.SafeZone;
import frc.robot.commands.ArmPositions.Stow;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;


public class RobotContainer {
    //subsytems
    public final Climber climber = new Climber();
    public final Intake intake = new Intake();
    public final Arm arm = new Arm();
    public final Elevator elevator = new Elevator();


    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);
    
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final SendableChooser<Command> AutonChooser = new SendableChooser<>();
    private final SlewRateLimiter eLimiter = new SlewRateLimiter(0.6);

    
    public RobotContainer() {
        //auto align
        NamedCommands.registerCommand("Align to Reef", new LockInReef(drivetrain));

        // arm positions
        // NamedCommands.registerCommand("Algae Processor", new AlgaeProccesor(elevator, arm));
        // NamedCommands.registerCommand("Algae Barge", new AlgeaBarge(elevator, arm));
        // NamedCommands.registerCommand("Algae L2", new AlgeaL2(elevator, arm));
        // NamedCommands.registerCommand("Algae L3", new AlgeaL3(elevator, arm));
        // NamedCommands.registerCommand("Coral L1", new CoralL1(elevator, arm));
        // NamedCommands.registerCommand("Coral L2", new CoralL2(elevator, arm));
        // NamedCommands.registerCommand("Coral L2", new CoralL3(elevator, arm));
        NamedCommands.registerCommand("Coral L4", new CoralL4(elevator, arm));
        NamedCommands.registerCommand("Stow", new CoralL1(elevator, arm));
        NamedCommands.registerCommand("intake down", intake.run(()->intake.setRotateSpeed(0.1)));
        // NamedCommands.registerCommand("Human Player Station", new HumanPlayerStation(elevator, arm));

        NamedCommands.registerCommand("Outtake Coral", arm.run(()->arm.setIntakeSpeed(-0.4)));
        NamedCommands.registerCommand("Intake Coral", arm.run(()->arm.setIntakeSpeed(0.8)));

        configureBindings();

        SmartDashboard.putData("AutonChooser", AutonChooser);
        AutonChooser.addOption("Left Auto", new PathPlannerAuto("left"));
        AutonChooser.addOption("Right Auto", new PathPlannerAuto("right"));
        AutonChooser.addOption("Center Auto", new PathPlannerAuto("center"));
    }

    private void configureBindings() {
        //////DRIVER CONTROLS
        //drive
        drivetrain.setDefaultCommand(new SwerveDrive(drivetrain, driverController, elevator));

        //climber
        driverController.leftTrigger().onTrue(climber.run(()-> climber.setClimberSpeed(-MathUtil.applyDeadband(driverController.getLeftTriggerAxis(), 0.1))));
        driverController.rightTrigger().onTrue(climber.run(()-> climber.setClimberSpeed(MathUtil.applyDeadband(driverController.getRightTriggerAxis(), 0.1))));
        //driverController.b().onTrue(climber.runOnce(()->climber.gotoPos(-0.6)));

        ///// END OF DRIVER CONTROLS
        
        
    
        /// OPERATOR CONTROLS
        arm.setDefaultCommand(arm.run(()->arm.manualControl
        (MathUtil.applyDeadband(-operatorController.getRightX(), 0.15), //rotate arm
        operatorController.leftTrigger(), //intake = true
        operatorController.rightTrigger(), //outtake = true
        operatorController.getLeftTriggerAxis(), //intake
        operatorController.getRightTriggerAxis()))); //outtake
        
        intake.setDefaultCommand(intake.run(()->intake.manualControl(
            operatorController.leftBumper(), // rotate forward = true
            operatorController.rightBumper(), //rotate backward = true
            0.1, // intake rotate speed
            operatorController.leftTrigger(), // intake = true
            operatorController.rightTrigger(), // outtake = true
            operatorController.getLeftTriggerAxis(), //intake
            operatorController.getRightTriggerAxis()))); //outtake

        // //raising and lowering elevator
        elevator.setDefaultCommand(elevator.run(()->elevator.setElevatorMotorSpeed(
            eLimiter.calculate(MathUtil.applyDeadband(operatorController.getLeftY(), 0.1)) // elevator up and down
            )));

        operatorController.button(8).whileTrue(elevator.runOnce(()->elevator.noStops()));
        operatorController.button(7).whileTrue(elevator.runOnce(()->elevator.newStop()));

        operatorController.a().onTrue(new Stow(elevator, arm));
        operatorController.b().onTrue(new SafeZone(elevator, arm));
    }

    public CommandXboxController getOperatorJoystick(){
        return driverController;
    }
    
    public CommandXboxController getDriverJoystick(){
        return operatorController;
    }

    public Command getAutonomousCommand() {
        return AutonChooser.getSelected();
    }
}
