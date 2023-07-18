package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.subsystems.DestemidosBot;

/**
 * OpMode focado em experimentação de novas idéias
 * e de medição das informações do robô
 */
@TeleOp(name = "TESTBOT", group = "Test")
@Disabled
public class FTC_TESTBOT extends CommandOpMode {
    private DestemidosBot robot;
    private GamepadEx player2;

    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot = new DestemidosBot(hardwareMap);
        robot.setBulkReadToAuto();

        player2 = new GamepadEx(gamepad2);

        /*register(robot.gripper, robot.armSystem);

        player2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenActive(new InstantCommand(() -> robot.gripper.moveWrist(1)))
                .whenInactive(new InstantCommand(() -> robot.gripper.moveWrist(0)));

        player2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenActive(new InstantCommand(() -> robot.gripper.moveWrist(-1)))
                .whenInactive(new InstantCommand(() -> robot.gripper.moveWrist(0)));

         */
    }

    @Override
    public void run() {
        double antes = System.currentTimeMillis();
        double energy = robot.voltageSensor.getVoltage();

        player2.readButtons();
        CommandScheduler.getInstance().run();

        robot.armSystem.setVoltage(energy);
        robot.drivetrain.updateVoltage(energy);

        robot.drivetrain.standardMecanumController(gamepad1);

        /*if(player2.gamepad.right_trigger > 0.0) {
            robot.gripper.gripper.setPower(player2.gamepad.right_trigger * 0.60);
        }

        if(player2.gamepad.left_trigger > 0.0) {
            robot.gripper.gripper.setPower(-player2.gamepad.left_trigger * 0.60);
        }

         */

        double depois = System.currentTimeMillis();

        telemetry.addData("arm position", robot.armSystem.armA.getCurrentPosition());
        telemetry.addData("arm current:", robot.armSystem.armA.getCurrent(CurrentUnit.MILLIAMPS));
        telemetry.addData("arm pid:", robot.armSystem.getArmPID());
        telemetry.addData("forearm position", robot.armSystem.forearmMotor.getCurrentPosition());
        telemetry.addData("forearm current:", robot.armSystem.forearmMotor.getCurrent(CurrentUnit.MILLIAMPS));
        telemetry.addData("forearm pid:", robot.armSystem.getForearmPID());

        telemetry.addData("voltagem do controlhub", robot.voltageSensor.getVoltage());
        telemetry.addData("tempo de loop (ms)", depois - antes);
        telemetry.update();
    }
}