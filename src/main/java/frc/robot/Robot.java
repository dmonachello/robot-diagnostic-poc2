package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;

import frc.robot.diag.core.DiagDeviceBase;
import frc.robot.diag.devices.LimitSwitchDiagDevice;
import frc.robot.diag.devices.TimerTerminator;
import frc.robot.diag.vendor.rev.SparkDiagDevice;

import com.revrobotics.spark.SparkLowLevel.MotorType;

public class Robot extends TimedRobot {

    private SparkDiagDevice shooterDev;
    private SparkDiagDevice intakeDev;
    private LimitSwitchDiagDevice limitTop;
    private TimerTerminator shootTimer;

    @Override
    public void robotInit() {

        // System.out.println("RobotInit: diag start");

        shooterDev = new SparkDiagDevice("ShooterMotor", 40, MotorType.kBrushless, 0.25);
        intakeDev  = new SparkDiagDevice("IntakeMotor",  41, MotorType.kBrushless, 0.20);

        limitTop   = new LimitSwitchDiagDevice("Limit switch Top", 9);
        shootTimer = new TimerTerminator("ShootTimer2s", 2.0);

        shooterDev.addTerminator(limitTop, true);
        shooterDev.addTerminator(shootTimer, false);

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
