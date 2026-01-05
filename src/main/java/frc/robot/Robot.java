package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;

import frc.robot.diag.core.DiagDeviceBase;
import frc.robot.diag.devices.LimitSwitchDiagDevice;
import frc.robot.diag.devices.TimerTerminator;
import frc.robot.diag.vendor.rev.SparkDiagDevice;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.diag.vendor.ctre.TalonDiagDevice;

public class Robot extends TimedRobot {

    private SparkDiagDevice shooterDev;
    private SparkDiagDevice intakeDev;
    private LimitSwitchDiagDevice limitTop;
    private TimerTerminator shootTimer;
    private TimerTerminator climbTimer;
    private TalonDiagDevice climberDev;   // pick a name that matches what it really is

    @Override
    public void robotInit() {

        // System.out.println("RobotInit: diag start");

        shooterDev = new SparkDiagDevice("ShooterMotor", 40, MotorType.kBrushless, 0.25);
        intakeDev  = new SparkDiagDevice("IntakeMotor",  41, MotorType.kBrushless, 0.20);

        climberDev = new TalonDiagDevice("ClimberMotor", 9, 0.25);


        limitTop   = new LimitSwitchDiagDevice("Limit switch Top", 9);
        shootTimer = new TimerTerminator("ShootTimer2s", 2.0);
        climbTimer = new TimerTerminator("ShootTimer2s", 3.0);

        shooterDev.addTerminator(limitTop, true);
        shooterDev.addTerminator(shootTimer, false);

        climberDev.addTerminator(limitTop, true);
        climberDev.addTerminator(climbTimer, false);  
        
        
        DiagDeviceBase.periodicAll();
        // System.out.println("RobotInit: diag end");
    }

    @Override
    public void teleopPeriodic() {
        DiagDeviceBase.periodicAll();
    }

    @Override
    public void disabledInit() {
        DiagDeviceBase.stopAll();
    }
}
