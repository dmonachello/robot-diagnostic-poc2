package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.diag.core.DiagDeviceBase;
import frc.robot.diag.devices.LimitSwitchDiagDevice;
import frc.robot.diag.devices.PositionTerminator;
import frc.robot.diag.devices.RotationTerminator;
import frc.robot.diag.devices.TimerTerminator;
import frc.robot.diag.vendor.rev.SparkDiagDevice;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.diag.vendor.ctre.TalonDiagDevice;

public class Robot extends TimedRobot {

    private SparkDiagDevice sparkGoodMotor;
    private SparkDiagDevice sparkBadMotor;
    private LimitSwitchDiagDevice limitTop;
    private TimerTerminator shootTimer;
    private TimerTerminator climbTimer;
    private PositionTerminator sparkGoodPosLimit;
    private RotationTerminator sparkGoodDeltaLimit;
    private TalonDiagDevice falconGoodMotor;   // pick a name that matches what it really is
    private boolean lastBrownedOut = false;
    private int brownoutCount = 0;

    @Override
    public void robotInit() {

        // System.out.println("RobotInit: diag start");

        sparkGoodMotor = new SparkDiagDevice("SparkGoodMotor", 40, MotorType.kBrushless, 0.25);
        sparkBadMotor  = new SparkDiagDevice("SparkBadMotor",  41, MotorType.kBrushless, 0.20);

        falconGoodMotor = new TalonDiagDevice("falconGoodMotor", 9, 0.25);


        limitTop   = new LimitSwitchDiagDevice("Limit switch Top", 9);
        shootTimer = new TimerTerminator("ShootTimer2s", 2.0);
        climbTimer = new TimerTerminator("ShootTimer2s", 3.0);

        sparkGoodMotor.addTerminator(limitTop, true);
        sparkGoodMotor.addTerminator(shootTimer, false);
        sparkGoodPosLimit = new PositionTerminator("SparkGoodPosLimit", 5.0);
        sparkGoodDeltaLimit = new RotationTerminator("SparkGoodDeltaLimit", 2.0);
        sparkGoodMotor.addTerminator(sparkGoodPosLimit, true);
        sparkGoodMotor.addTerminator(sparkGoodDeltaLimit, false);

        falconGoodMotor.addTerminator(limitTop, true);
        falconGoodMotor.addTerminator(climbTimer, false);  
        
        
        DiagDeviceBase.periodicAll();
        // System.out.println("RobotInit: diag end");
    }

    @Override
    public void teleopPeriodic() {
        DiagDeviceBase.periodicAll();
    }

    @Override
    public void robotPeriodic() {
        DiagDeviceBase.dashboardAll(DriverStation.isTeleopEnabled());
        boolean brownedOut = RobotController.isBrownedOut();
        if (brownedOut && !lastBrownedOut) {
            brownoutCount++;
        }
        lastBrownedOut = brownedOut;
        SmartDashboard.putNumber("Diag/BrownoutCount", brownoutCount);
    }

    @Override
    public void disabledInit() {
        DiagDeviceBase.stopAll();
    }
}
