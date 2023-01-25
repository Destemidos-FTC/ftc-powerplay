package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.hardware.DestemidosHardware;
@Config
@TeleOp(name = "Teste Braço Autonomo", group = "Test")

public class TesteBraçoAutonomo extends OpMode {
    private DestemidosHardware robo;
    private PIDController controller;

    public static double p = 0, i = 0, d = 0;
    public static double f = 0;

    public static int target = 0;

    private final double ticks_to_degree = 1120.0 / 360.0;
    private final int    degree_to_ticks = 360 / 1120;

    @Override
    public void init() {
        robo = new DestemidosHardware(hardwareMap);
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    }
       
    @Override
    public void loop(){

        // recebemos a posição do motor em ticks
        int motorBraçoACurrentPosition = robo.motorBraçoA.getCurrentPosition();
        int motorBraçoBCurrentPosition = robo.motorBraçoB.getCurrentPosition();

        // convertemos ticks em graus
        double braçoA_in_degree = motorBraçoACurrentPosition;

        // nosso setpoint e valor medido será em graus
        double pid = controller.calculate(braçoA_in_degree, target);
        double ff = Math.cos(Math.toRadians(target * ticks_to_degree)) * f;

        double power = pid; //+ ff;

        int targetA = target * degree_to_ticks;
        int targetB = target * degree_to_ticks;

        robo.motorBraçoA.setTargetPosition(targetA);
        robo.motorBraçoB.setTargetPosition(targetB);

        robo.motorBraçoA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robo.motorBraçoB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robo.motorBraçoA.setPower(power / 2);
        robo.motorBraçoB.setPower(power / 2);

        telemetry.addData("BraçoA - Pos", motorBraçoACurrentPosition);
        telemetry.addData("BraçoB - Pos", motorBraçoBCurrentPosition);
        telemetry.addData("Target", target);
        telemetry.addData("valor do ff: ", ff);
        telemetry.update();
    }
}

