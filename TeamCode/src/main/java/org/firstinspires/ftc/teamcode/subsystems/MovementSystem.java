package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.hardware.DestemidosBot;
import org.firstinspires.ftc.teamcode.hardware.RobotConstants;

/*
 * MovementSystem - módulo responsável por agrupar os
 * diversos modos de movimentação que usamos e experimentamos
 */

public final class MovementSystem {
    private final DestemidosBot robot;
    private DRIVE_MODE driveMode;

    enum DRIVE_MODE {
        STANDARD,
        PRECISE,
        FIELD_ORIENTED,
        TANK
    }

    public MovementSystem(DestemidosBot robot) {
        this.robot = robot;
        this.driveMode = DRIVE_MODE.STANDARD;
    }

    public void setDriveMode(DRIVE_MODE driveMode) {
        this.driveMode = driveMode;
    }

    public DRIVE_MODE getDriveMode() {
        return driveMode;
    }

    public void driveRobot(Gamepad driver) {
        switch (driveMode) {
            case STANDARD:
                standardMecanumController(driver);

            case PRECISE:
                //preciseMecanumController(driver);

            case TANK:
                tankController(driver);

            case FIELD_ORIENTED:
                fieldOrientedController(driver);

            default:
                standardMecanumController(driver);
        }
    }

    public void tankController(Gamepad driver) {
        double joystick_y  = -driver.left_stick_y;
        double giro        = -driver.right_stick_x;

        double denominador = Math.max( Math.abs(joystick_y) + Math.abs(giro), 1);

        double direitaFrentePower   = (joystick_y - giro) / denominador;
        double direitaTrasPower     = (joystick_y - giro) / denominador;
        double esquerdaFrentePower  = (joystick_y + giro) / denominador;
        double esquerdaTrasPower    = (joystick_y + giro) / denominador;

    robot.drivetrain.setMotorsPower(
        direitaFrentePower, 
        direitaTrasPower, 
        esquerdaFrentePower, 
        esquerdaTrasPower);
    }
    // Movimentação padrão e mais refinada do nosso controle, baseada no vídeo
    // extremamente didático do Gavin Ford: https://youtu.be/gnSW2QpkGXQ
    public void standardMecanumController(Gamepad driver)
    {
        double joystick_y  = driver.left_stick_y;  //* RobotConstants.MAX_SPEED;
        double joystick_x  = -driver.left_stick_x; // * RobotConstants.MAX_SPEED; //* RobotConstants.kCorretorJoystickX;
        double giro        = -driver.right_stick_x; //* RobotConstants.MAX_SPEED;

        // o denominador sempre será a maior força (valor absoluto) entre os 4 motores, ou equivalente a 1.
        // isso permite que todos mantenham a mesma taxa, mesmo que um motor ultrapasse os limites [-1, 1]
        //double denominador = Math.abs(joystick_y) + Math.abs(joystick_x) + Math.abs(giro);

        //if(denominador > 1) {
            double direitaFrentePower   = (joystick_y - joystick_x - giro); // denominador;
            double direitaTrasPower     = (joystick_y + joystick_x - giro); // denominador;
            double esquerdaFrentePower  = (joystick_y + joystick_x + giro); // denominador;
            double esquerdaTrasPower    = (joystick_y - joystick_x + giro); // denominador;

            robot.drivetrain.setMotorsPower(direitaFrentePower, direitaTrasPower, esquerdaFrentePower, esquerdaTrasPower);
        //}
    }

    // Controle possivelmente mais preciso, baseado no método 2 do youtuber Gavin Ford
    public void preciseMecanumController(double theta, double direction, double turn) {

        double seno         = Math.sin(theta - RobotConstants.MECANUM_WHEELS_ANGLE);
        double cosseno      = Math.cos(theta - RobotConstants.MECANUM_WHEELS_ANGLE);

        // neste caso, o denominador vai "normalizar" os valores de cada eixo
        double denominador  = Math.max( Math.abs(seno), Math.abs(cosseno) );

        double direitaFrentePower   = (seno / denominador) - turn;
        double direitaTrasPower     = (cosseno / denominador) - turn;
        double esquerdaFrentePower  = (cosseno / denominador) + turn;
        double esquerdaTrasPower    = (seno / denominador) + turn;

        if(direction + Math.abs(turn) > 1) {
            direitaFrentePower  /= direction + Math.abs(turn);
            direitaTrasPower    /= direction + Math.abs(turn);
            esquerdaFrentePower /= direction + Math.abs(turn);
            esquerdaTrasPower   /= direction + Math.abs(turn);
        }

        robot.drivetrain.setMotorsPower(
            direction * direitaFrentePower,
            direction * direitaTrasPower,
            direction * esquerdaFrentePower,
            direction * esquerdaTrasPower);
    }
    public void controleMecanumAlternativo(Gamepad driver, DestemidosBot robot) {
        double direction = Math.hypot(driver.left_stick_x, driver.left_stick_y);
        double robotAngle = Math.atan2(driver.left_stick_y, driver.left_stick_x) - Math.PI / 4;
        double rightX = driver.right_stick_x;

        final double v1 = direction * Math.cos(robotAngle) + rightX;
        final double v2 = direction * Math.sin(robotAngle) - rightX;
        final double v3 = direction * Math.sin(robotAngle) + rightX;
        final double v4 = direction * Math.cos(robotAngle) - rightX;

        robot.drivetrain.setMotorsPower(v2, v4, v1, v3);
    }

    // um controle que sempre direciona o robô para onde apontamos no joystick, independente
    // da orentação do robô na arena
    public void fieldOrientedController(Gamepad driver) {
        double joystick_y  = -driver.left_stick_y  * RobotConstants.MAX_SPEED;
        double joystick_x  = -driver.left_stick_x  * RobotConstants.MAX_SPEED; //* RobotConstants.kCorretorJoystickX;
        double giro        = -driver.right_stick_x * RobotConstants.MAX_SPEED;

        double botHeading = robot.drivetrain.getSensorIMU().getRobotOrientation(
                AxesReference.INTRINSIC,
                AxesOrder.XYZ,
                AngleUnit.RADIANS
        ).thirdAngle;

        double rotationX = joystick_x * Math.cos(botHeading) - joystick_y * Math.sin(botHeading);
        double rotationY = joystick_x * Math.sin(botHeading) + joystick_y * Math.cos(botHeading);

        double denominador = Math.max( Math.abs(joystick_y) + Math.abs(joystick_x) + Math.abs(giro), 1);

        double direitaFrentePower   = (rotationY - rotationX - giro) / denominador;
        double direitaTrasPower     = (rotationY + rotationX - giro) / denominador;
        double esquerdaFrentePower  = (rotationY + rotationX + giro) / denominador;
        double esquerdaTrasPower    = (rotationY - rotationX + giro) / denominador;

        robot.drivetrain.setMotorsPower(direitaFrentePower, direitaTrasPower, esquerdaFrentePower, esquerdaTrasPower);
    }

}