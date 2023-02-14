package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.DestemidosBot;
import org.firstinspires.ftc.teamcode.hardware.RobotConstants;
import org.firstinspires.ftc.teamcode.subsystems.ArmSystem;
import org.firstinspires.ftc.teamcode.subsystems.MovementSystem;
import org.firstinspires.ftc.teamcode.utils.RobotLogger;

@TeleOp(name="TESTBOT", group = "Test")
public class FTC_TESTBOT extends LinearOpMode {
    private DestemidosBot robot;
    private double[] motorVelocities = new double[4];

    @Override
    public void runOpMode() throws InterruptedException {

        // setup the hardware components
        robot = new DestemidosBot(hardwareMap);
        MovementSystem movementSystem = new MovementSystem(robot);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        while(opModeIsActive()) {
            //robot.setBulkReadToAuto();

            // movimentação padrão das partidas
            movementSystem.standardMecanumController(gamepad1);

            for (int i = 0; i < robot.drivetrain.getMotors().size(); i++) {
                motorVelocities[i] = robot.drivetrain.getMotors().get(i).getVelocity();
            }

            // Debug de informações
            RobotLogger.debugControles(telemetry, gamepad1, gamepad2);
            telemetry.addData("velocidade - motor 0", motorVelocities[0]);
            telemetry.addData("velocidade - motor 1", motorVelocities[1]);
            telemetry.addData("velocidade - motor 2", motorVelocities[2]);
            telemetry.addData("velocidade - motor 3", motorVelocities[3]);
            telemetry.update();
        }
    }
}