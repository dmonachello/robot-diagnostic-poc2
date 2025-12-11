package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;

import frc.robot.diag.core.DiagDeviceBase;
import frc.robot.diag.devices.LimitSwitchDiagDevice;
import frc.robot.diag.devices.TimerTerminator;
import frc.robot.diag.vendor.rev.SparkDiagDevice;

import com.revrobotics.spark.SparkLowLevel.MotorType;

public class Robot extends TimedRobot {

    private SparkDiagDevice        shooterDev;
    private SparkDiagDevice        intakeDev;
    private LimitSwitchDiagDevice  limitTop;
    private TimerTerminator        shootTimer;

    @Override
    public void robotInit() {

        System.out.println("RobotInit: diag 1 start");

        shooterDev = new SparkDiagDevice("ShooterMotor", 40, MotorType.kBrushless, 0.25);
        intakeDev  = new SparkDiagDevice("IntakeMotor",  41, MotorType.kBrushless, 0.20);

        limitTop   = new LimitSwitchDiagDevice("Limit switch Top", 9);
        shootTimer = new TimerTerminator("ShootTimer2s", 2.0);

        // use limit switch and optional timer as terminators for shooter

        // Limit switch: armed by default, label taken from getTerminatorName()
        shooterDev.addTerminator(limitTop);

        // Timer: label "ShootTimer2s", default UseTerm_* = false
        shooterDev.addTerminator(shootTimer, "ShootTimer2s", false);

        System.out.println("RobotInit: diag 1 end");
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
