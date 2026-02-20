package org.firstinspires.ftc.teamcode.TeleOp;

import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;


@Config
public class FlywheelPIDTest {

    DcMotorEx flyMaster;
    DcMotorEx flySlave;
    Telemetry telemetry;

    static final double TICKS_PER_REV = 28.0;
    static final double MAX_RPM = 6000.0;
    static final double MAX_TPS = MAX_RPM * TICKS_PER_REV / 60.0;

    public static double targetRPM = 3000;
    double targetTPS = rpmToTicks(targetRPM);



    public static double kP = 67.0;
    public static double kI = 2.0;
    public static double kD = 1.0;
    public static double kF = 0.00022;

    double masterIntegral = 0;
    double slaveIntegral = 0;
    double lastMasterError = 0;
    double lastSlaveError = 0;

    long lastTime = 0;
    FtcDashboard dashboard;

    public void init(HardwareMap hw, Telemetry telemetry) {

        dashboard = FtcDashboard.getInstance();
        this.telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        this.telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        flyMaster = hw.get(DcMotorEx.class, "l1");
        flySlave = hw.get(DcMotorEx.class, "l2");

        flyMaster.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flySlave.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        flyMaster.setDirection(DcMotorSimple.Direction.FORWARD);
        flySlave.setDirection(DcMotorSimple.Direction.REVERSE);

        flyMaster.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flySlave.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        MotorConfigurationType m = flyMaster.getMotorType().clone();
        m.setAchieveableMaxRPMFraction(1.0);
        flyMaster.setMotorType(m);

        MotorConfigurationType s = flySlave.getMotorType().clone();
        s.setAchieveableMaxRPMFraction(1.0);
        flySlave.setMotorType(s);

        PIDFCoefficients pidfMaster = new PIDFCoefficients(kP, kI, kD, kF);
        PIDFCoefficients pidfSlave = new PIDFCoefficients(kP, kI, kD, kF);

        flyMaster.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfMaster);
        flySlave.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfSlave);
    }

    public void loop(boolean trigger) {
        double targetTPS = rpmToTicks(targetRPM);

        PIDFCoefficients pidf = new PIDFCoefficients(kP, kI, kD, kF);
        flyMaster.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        flySlave.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);

        long currentTime = System.nanoTime();
        double dt = (currentTime - lastTime) / 1e9;
        lastTime = currentTime;
        if (dt <= 0) dt = 0.001;

        if (trigger) {
            flyMaster.setVelocity(targetTPS);
            flySlave.setVelocity(targetTPS);
        } else {
            flyMaster.setVelocity(0);
            flySlave.setVelocity(0);
            masterIntegral = 0;
            slaveIntegral = 0;
            lastMasterError = 0;
            lastSlaveError = 0;
        }
        double masterTPS = flyMaster.getVelocity();
        double masterRPM = ticksToRPM(masterTPS);
        double slaveTPS = flySlave.getVelocity();
        double slaveRPM = ticksToRPM(slaveTPS);

        TelemetryPacket packet = new TelemetryPacket();

        packet.put("TargetRPM", targetRPM);
        packet.put("MasterRPM", masterRPM);
        packet.put("SlaveRPM", slaveRPM);
        packet.put("Error", targetRPM - masterRPM);
        packet.put("Power", flyMaster.getPower());

        dashboard.sendTelemetryPacket(packet);

    }

    static double rpmToTicks(double rpm) {
        return rpm * TICKS_PER_REV / 60.0;
    }

    static double ticksToRPM(double tps) {
        return tps * 60.0 / TICKS_PER_REV;
    }
    public boolean isAtSpeed(double toleranceRPM) {
        double masterRPM = ticksToRPM(flyMaster.getVelocity());
        double slaveRPM  = ticksToRPM(flySlave.getVelocity());

        return Math.abs(masterRPM - targetRPM) < toleranceRPM
                && Math.abs(slaveRPM  - targetRPM) < toleranceRPM;
    }
    public void brake() {
        flyMaster.setPower(-0.3);
        flySlave.setPower(-0.3);
        sleep(100);
        flyMaster.setPower(0);
        flySlave.setPower(0);
        flyMaster.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flySlave.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}