package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.DestemidosBot;
import org.firstinspires.ftc.teamcode.subsystems.GripSystem;
import org.firstinspires.ftc.teamcode.subsystems.MovementSystem;


@TeleOp(name = "FTC_4", group = "TeleOps")
public class FTC_4 extends LinearOpMode {
    private DestemidosBot robot;

    @Override
    public void runOpMode() {
        robot = new DestemidosBot(hardwareMap);
        MovementSystem movementSystem = new MovementSystem(robot);

        waitForStart();
        while (opModeIsActive()) {
            robot.setBulkReadToAuto();

            // movimentação padrão das partidas
            movementSystem.standardMecanumController(gamepad1);

            // controles do braço e da mão
            robot.armSystem.moveArms(gamepad2);

            GripSystem.coletarCones(gamepad2, robot);
        }
    }
}