// Property of: CANQUEENS-25668
package org.firstinspires.ftc.teamcode.CANQUEENS;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

// Anotación que indica que este es un opmode de tipo TeleOp en el grupo Linear OpMode
@TeleOp(name="CANQUEENS_teleop", group="Linear OpMode")
public class TeleOp_CANQUEENS extends LinearOpMode {

    // Tiempo transcurrido desde el inicio del opmode
    private ElapsedTime runtime = new ElapsedTime();
    
    // Motores del robot
    private DcMotor leftFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightFrontDrive;
    private DcMotor rightBackDrive;

    // Método que se ejecuta cuando se inicia el opmode
    @Override
    public void runOpMode() {

        // Inicialización de los motores usando el hardware map
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back");

        // Configuración de la dirección de los motores
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // Mensaje de inicialización en el telemetría
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Espera a que el opmode sea iniciado
        waitForStart();
        runtime.reset();  // Reinicia el cronómetro

        // Bucle principal del opmode que se ejecuta mientras el opmode esté activo
        while (opModeIsActive()) {
            double max;

            // Obtención de las entradas del gamepad
            double axial   = -gamepad1.left_stick_y;   // Movimiento hacia adelante/atrás
            double lateral =  gamepad1.left_stick_x;   // Movimiento hacia los lados
            double yaw     =  gamepad1.right_stick_x;  // Rotación

            // Cálculo de la potencia de cada motor basada en las entradas del gamepad
            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;

            // Normalización de las potencias para asegurar que no excedan 1.0
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }

            // Ajuste de la potencia de cada motor
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            // Actualización del telemetría con el tiempo de ejecución
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}