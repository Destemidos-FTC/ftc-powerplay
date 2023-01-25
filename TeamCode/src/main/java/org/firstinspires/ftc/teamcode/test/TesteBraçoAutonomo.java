package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.hardware.DestemidosHardware;
@Config
@TeleOp(name = "Teste Braço Autonomo", group = "Test")

public class TesteBraçoAutonomo extends OpMode {
    private DestemidosHardware robo;
    private PIDController controller;

    public static double p = 0, i = 0, d = 0;
    public static double f = 0;

    public static int target = 0;

    private final double ticks_in_degree = 700.0 / 180.0;

    private DcMotorEx motorBraçoA;
    private DcMotorEx motorBraçoB;

    @Override
    public void init() {
    
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        motorBraçoA = hardwareMap.get(DcMotorEx.class, "motorBraçoA");
        motorBraçoB = hardwareMap.get(DcMotorEx.class, "motorBraçoB");
        motorBraçoA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBraçoB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        motorBraçoA.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBraçoB.setDirection(DcMotorSimple.Direction.REVERSE);
    
    }
       
    @Override
    public void loop(){
        
        controller.setPID (p, i, d);
        int braçoPosi = motorBraçoA.getCurrentPosition();
        double pid = controller.calculate(braçoPosi, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

        double power = pid + ff;

        motorBraçoA.setPower(power);
        motorBraçoB.setPower(power);

        telemetry.addData("Pos", braçoPosi);
        telemetry.addData("Target", target);
        telemetry.update();
    }
}

