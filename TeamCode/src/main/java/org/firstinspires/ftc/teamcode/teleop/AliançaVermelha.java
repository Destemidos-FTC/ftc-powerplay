package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.DestemidosBot;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

@TeleOp
public class
AliançaVermelha extends CommandOpMode {
    private Drivetrain drivetrain;

    private DestemidosBot robot;
    private GamepadEx player2;

    @Override
    public void initialize() {
        drivetrain = new Drivetrain(hardwareMap);
        robot = new DestemidosBot(hardwareMap);
        robot.setBulkReadToAuto();

        player2 = new GamepadEx(gamepad2);

        CommandScheduler.getInstance().reset();


        player2.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenActive(new InstantCommand(() -> robot.servo.moveMonheca(1)))
                .whenInactive(new InstantCommand(() -> robot.servo.moveMonheca(0)));

        player2.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenActive(new InstantCommand(() -> robot.servo.moveMonheca(-1)))
                .whenInactive(new InstantCommand(() -> robot.servo.moveMonheca(0)));

        player2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenActive(new InstantCommand(() -> robot.servo.moveWrist(1)))
                .whenInactive(new InstantCommand(() -> robot.servo.turnOffWrist()));

        player2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenActive(new InstantCommand(() -> robot.servo.moveWrist(-1)))
                .whenInactive(new InstantCommand(() -> robot.servo.turnOffWrist()));

        /*player2.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new ArmToGround(robot));

        player2.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new ArmToLowJunction(robot));

        player2.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new ArmToMediumJunction(robot));

        player2.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new ArmToHighJunction(robot));*/

    }


        @Override
        public void run() {
            CommandScheduler.getInstance().run();
            player2.readButtons();

        /*if(player2.gamepad.right_trigger > 0.0) {
            robot.gripper.gripper.setPower(player2.gamepad.right_trigger * 0.60);
        }

        if(player2.gamepad.left_trigger > 0.0) {
            robot.gripper.gripper.setPower(-player2.gamepad.left_trigger * 0.60);
        }*/

            robot.drivetrain.standardMecanumController(gamepad1);
            robot.simpleArm.forceArm(player2.gamepad.right_stick_y);
        }


    }

