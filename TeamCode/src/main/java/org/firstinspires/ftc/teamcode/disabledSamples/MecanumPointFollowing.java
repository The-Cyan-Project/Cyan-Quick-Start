package org.firstinspires.ftc.teamcode.disabledSamples;

import com.github.bouyio.cyancore.debugger.DebugPacket;
import com.github.bouyio.cyancore.debugger.Debuggers;
import com.github.bouyio.cyancore.debugger.Logger;
import com.github.bouyio.cyancore.geomery.Point;
import com.github.bouyio.cyancore.localization.ThreeDeadWheelOdometry;
import com.github.bouyio.cyancore.pathing.PointSequence;
import com.github.bouyio.cyancore.pathing.engine.MecanumDriveVectorInterpreter;
import com.github.bouyio.cyancore.pathing.engine.PathFollower;
import com.github.bouyio.cyancore.util.Distance;
import com.github.bouyio.cyancore.util.PIDCoefficients;
import com.github.bouyio.cyancore.util.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * <p>
 *     This class demonstrates path following in CyanFTC.
 *     The way path following differs from point sequencing is that it much smoother.
 *     This is achieved by dynamically adjusting the targeted point.
 * </p>
 * */
@Disabled
@TeleOp()
public class MecanumPointFollowing extends OpMode {

    // TODO: Set the path following configurations.
    // NOTE: Tuning look ahead distance is very simple.
    // The greater the value is smoother the following gets
    // at the expense of some accuracy.
    final double LOOK_AHEAD_DISTANCE = 000000000;

    // NOTE: Once again the greater the value of admissible
    // point error is the smoother the following gets at the
    // expense of some accuracy.
    final double ADMISSIBLE_POINT_ERROR = 000000000;

    // TODO: Set constants to match the ones of the robot
    // NOTE: The distance unit of the track width must be
    // the same as the distance unit of the converted motor
    // rotations.
    final double ENCODER_WIDTH = 0000000000;
    final double WHEEL_RADIUS = 000000000;
    final double ENCODER_COUNT_PER_REVOLUTION = 000000000;
    final double TICKS_TO_LINEAR_DISTANCE = 2 * Math.PI * WHEEL_RADIUS / ENCODER_COUNT_PER_REVOLUTION;

    /**The localization system.*/
    ThreeDeadWheelOdometry odometry;

    // The encoders we will be using.
    DcMotor leftParallelEncoder;
    DcMotor rightParallelEncoder;
    DcMotor perpendicularEncoder;

    /**The sequence following system.*/
    PathFollower follower;

    // The motors we will be using.
    DcMotor leftFrontMotor;
    DcMotor leftBackMotor;
    DcMotor rightFrontMotor;
    DcMotor rightBackMotor;

    /**The logging system.*/
    Logger logger;

    /**The point used for point following demonstration.*/
    Point point;

    @Override
    public void init() {
        // Initializing hardware.
        leftFrontMotor = hardwareMap.get(DcMotor.class, "left_front_motor");
        leftBackMotor = hardwareMap.get(DcMotor.class, "left_back_motor");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "right_front_motor");
        rightBackMotor = hardwareMap.get(DcMotor.class, "right_back_motor");
        rightFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftParallelEncoder = hardwareMap.get(DcMotor.class, "left_y_encoder");
        rightParallelEncoder = hardwareMap.get(DcMotor.class, "right_y_encoder");
        perpendicularEncoder = hardwareMap.get(DcMotor.class, "x_encoder");

        // TODO: Reverse if necessary.
        //leftParallelEncoder.setDirection(DcMotorSimple.Direction.REVERSE);
        //rightParallelEncoder.setDirection(DcMotorSimple.Direction.REVERSE);
        //perpendicularEncoder.setDirection(DcMotorSimple.Direction.REVERSE);


        // Initializing the measurement provider for the localization system.
        ThreeDeadWheelOdometry.MeasurementProvider measurementProvider = new ThreeDeadWheelOdometry.MeasurementProvider(
                perpendicularEncoder::getCurrentPosition,
                leftParallelEncoder::getCurrentPosition,
                rightParallelEncoder::getCurrentPosition,
                TICKS_TO_LINEAR_DISTANCE
        );

        // Initializing the localization system itself.
        odometry = new ThreeDeadWheelOdometry(ENCODER_WIDTH, Distance.DistanceUnit.CM, measurementProvider);

        // Initializing the sequence following system.

        // Initializing the steering PID controller coefficients.
        // TODO: Tune the controller and set the actual coefficient.
        PIDCoefficients coefficients = new PIDCoefficients();
        coefficients.kP = 0000000;
        coefficients.kD = 0000000;
        coefficients.kI = 0000000;

        // Initializing the vector interpreter.
        // TODO: See if the reverse fits your use case.
        // TODO: If not change the constructor parameter to false.
        MecanumDriveVectorInterpreter vectorInterpreter =
                new MecanumDriveVectorInterpreter();

        // Initializing the path follower itself.
        follower = new PathFollower(odometry, vectorInterpreter, new PIDController(coefficients));

        // Configuring follower.
        // TODO: Set the preferred distance unit.
        follower.purePursuitSetUp(LOOK_AHEAD_DISTANCE, ADMISSIBLE_POINT_ERROR);
        follower.setDistanceUnitOfMeasurement(Distance.DistanceUnit.CM);

        // Initializing the point.
        // TODO: Set the actual coordinates of the point.
        point = new Point(000000, 000000);

        // Initializing localization system logging.
        logger = Debuggers.getGlobalLogger();
        odometry.attachLogger(logger);
        follower.attachLogger(logger);

    }

    @Override
    public void loop() {

        // Point following code.

        // Setting the target.
        follower.followPoint(point);

        // Getting the movement instruction from the follower.
        double[] motorPowers = follower.getCalculatedPowers();

        // Applying the motor powers.
        leftFrontMotor.setPower(motorPowers[MecanumDriveVectorInterpreter.LEFT_FRONT_MOTOR_ID]);
        leftBackMotor.setPower(motorPowers[MecanumDriveVectorInterpreter.LEFT_BACK_MOTOR_ID]);
        rightFrontMotor.setPower(motorPowers[MecanumDriveVectorInterpreter.RIGHT_FRONT_MOTOR_ID]);
        rightBackMotor.setPower(motorPowers[MecanumDriveVectorInterpreter.RIGHT_BACK_MOTOR_ID]);

        // Updating the debug information.
        odometry.debug();
        follower.debug();

        // Retrieving and displaying the debug information.
        DebugPacket[] loggerPackets = logger.dump();

        for (DebugPacket packet : loggerPackets) {
            telemetry.addData(packet.getHeader().getIdentifier(), packet.getValue());
        }
    }
}
