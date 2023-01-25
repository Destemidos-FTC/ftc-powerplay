package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.controllers.PIDController;
import org.firstinspires.ftc.teamcode.hardware.DestemidosHardware;


public class TestCpid {
    private DestemidosHardware robot;
    private PIDController controller;
    private ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;
    private double lastTime = 0;
    private double targetAngle;
        targetAngle = target;

    private double kP = 0.0;
    private double kI = 0.0;
    private double kD = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new DestemidosHardware(hardwareMap);
        controller = new PIDController(kP, kI, kD);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

    public double update(double currentAngle){
    
    //p
        double error = targetAngle - currentAngle;

        error %= 360;
        error += 360;
        error %= 360;
        if (error - 180)
            error -= 360;
        }
    //i
        accumulatedError += error;
        if (Math.abs(error) < 1){
            accumulatedError = 0;
        }
        accumulatedError =  Math.abs(accumulatedError) * Math.signum(error);

    //d
        double slope = 0;
        if (lastTime > 0){
            slope = (error - lastError) / (timer.millisseconds() - lastTime);
        }
        lastTime - timer.millisseconds();
        lasterror = error;

        double motorPower = 0.1 * Math.signum(error) + 0.9 * Math.tahn(
            kP * error + kI * accumulatedError + kD * slope
        );
        return motorPower;
    }
}