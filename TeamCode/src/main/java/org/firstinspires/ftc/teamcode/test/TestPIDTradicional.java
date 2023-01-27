package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.DestemidosBot;

@Config
@TeleOp(name = "TestPID Tradicional")
public class TestPIDTradicional extends LinearOpMode {
    private DestemidosBot robot;
    private PIDController controller;
    private ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;
    private double lastTime = 0;
    private double accumulatedError = 0.0;

    // váriáveis modificadas pelo ftc-dashboard
    public static double targetAngle;
    public static double kP = 0.0;
    public static double kI = 0.0;
    public static double kD = 0.0;

    public TestPIDTradicional(double targetAngle, double v, int i, double v1) {
    }

    private double updatePID(double currentAngle) {
        //p
        double error = targetAngle - currentAngle;
        error %= 360;
        error += 360;
        error %= 360;
        if (error == 180) { error -= 360; }

        //i
        accumulatedError += error;
        if (Math.abs(error) < 1){
            accumulatedError = 0;
        }

        accumulatedError =  Math.abs(accumulatedError) * Math.signum(error);

        //d
        double slope = 0;
        if (lastTime > 0){
            slope = (error - lastError) / (timer.milliseconds() - lastTime);
        }

        lastTime = timer.milliseconds();
        lastError = error;

        return 0.1 * Math.signum(error) + 0.9 * Math.tanh(
                kP * error + kI * accumulatedError + kD * slope) ;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new DestemidosBot(hardwareMap);
        controller = new PIDController(kP, kI, kD);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
    }

    public double getLastSlope() {
        return 0;
    }

    public double update(double absoluteAngle) {
        return absoluteAngle;
    }
}