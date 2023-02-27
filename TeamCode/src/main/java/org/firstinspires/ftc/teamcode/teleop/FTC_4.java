package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.DestemidosBot;

/**
 * OpMode princiapl para as competições.
 * O foco é sempre buscar o máximo de performance
 * e estabilidade por aqui
 */
@TeleOp(name = "FTC_4", group = "TeleOps")
public class FTC_4 extends CommandOpMode {
    private DestemidosBot robot;
    @Override
    public void initialize() {
        // preparando o sistema de comandos
        CommandScheduler.getInstance().reset();

        // configurçãoes do robô abaixo
        robot = new DestemidosBot(hardwareMap);
        robot.setBulkReadToAuto();
    }

    @Override
    public void run() {
        double start_loop = System.currentTimeMillis();

        // controle da base
        robot.drivetrain.standardMecanumController(gamepad1);

        // controle do intake

        // controle do outtake

        // por fim, executa todos os sistemas
        CommandScheduler.getInstance().run();

        double end_loop = System.currentTimeMillis();
        telemetry.addData("loop time", (end_loop - start_loop));
        telemetry.update();
    }
}