package org.firstinspires.ftc.teamcode.disabledSamples;

import com.github.bouyio.cyancore.debugger.DebugPacket;
import com.github.bouyio.cyancore.debugger.Debuggers;
import com.github.bouyio.cyancore.debugger.Logger;
import com.github.bouyio.cyancore.geomery.Point;
import com.github.bouyio.cyancore.localization.TankKinematics;
import com.github.bouyio.cyancore.pathing.PointSequence;
import com.github.bouyio.cyancore.pathing.engine.PathFollower;
import com.github.bouyio.cyancore.pathing.engine.TankDriveVectorInterpreter;
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
 *     This class demonstrates <strong>sequential</strong> point following in CyanFTC.
 * </p>
 * */
@Disabled
@TeleOp()
public class TankPointSequencing extends OpMode {

    // TODO: Set constants to match the ones of the robot
    // NOTE: The distance unit of the track width must be
    // the same as the distance unit of the converted motor
    // rotations.
    final double TRACK_WIDTH = 0000000000;
    final double WHEEL_RADIUS = 000000000;
    final double ENCODER_COUNT_PER_REVOLUTION = 000000000;
    final double TICKS_TO_LINEAR_DISTANCE = 2 * Math.PI * WHEEL_RADIUS / ENCODER_COUNT_PER_REVOLUTION;

    /**The localization system.*/
    TankKinematics odometry;

    /**The sequence following system.*/
    PathFollower follower;

    // The motors we will be using.
    DcMotor leftMotor;
    DcMotor rightMotor;

    /**The logging system.*/
    Logger logger;

    /**The sequence used for point following demonstration.*/
    PointSequence sequence;

    @Override
    public void init() {
        // Initializing hardware.
        leftMotor = hardwareMap.get(DcMotor.class, "left_motor");
        rightMotor = hardwareMap.get(DcMotor.class, "right_motor");
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Initializing localization system.
        TankKinematics.MeasurementProvider measurementProvider = new TankKinematics.MeasurementProvider(
                leftMotor::getCurrentPosition,
                rightMotor::getCurrentPosition,
                TICKS_TO_LINEAR_DISTANCE
        );

        odometry = new TankKinematics(TRACK_WIDTH, Distance.DistanceUnit.CM, measurementProvider);

        // Initializing the steering PID controller coefficients.
        // TODO: Tune the controller and set the actual coefficient.
        PIDCoefficients coefficients = new PIDCoefficients();
        coefficients.kP = 0000000;
        coefficients.kD = 0000000;
        coefficients.kI = 0000000;

        // Initializing the vector interpreter.
        // TODO: See if the reverse fits your use case.
        // TODO: If not change the constructor parameter to false.
        TankDriveVectorInterpreter vectorInterpreter =
                new TankDriveVectorInterpreter(true);

        // Initializing the sequence follower itself.
        follower = new PathFollower(odometry, vectorInterpreter, new PIDController(coefficients));

        // Configuring follower.
        // TODO: Set the preferred distance unit and error tolerance.
        follower.setDistanceErrorTolerance(0000000);
        follower.setDistanceUnitOfMeasurement(Distance.DistanceUnit.CM);

        // Initializing the sequence.
        // TODO: Set the actual unit of measurement of the sequence.
        // TODO: Set the actual coordinates of the points.
        // TODO: Use as many point as it is necessary.
        sequence = new PointSequence(
                new Point(0000000, 0000000),
                new Point(0000000, 0000000),
                new Point(0000000, 0000000),
                new Point(0000000, 0000000),
                new Point(0000000, 0000000)
        );

        sequence.setUnitOfMeasurement(Distance.DistanceUnit.CM);

        // Initializing localization system logging.
        logger = Debuggers.getGlobalLogger();
        odometry.attachLogger(logger);
        follower.attachLogger(logger);

    }

    @Override
    public void loop() {

        // Point following code.

        // Setting the target.
        follower.followPointSequence(sequence);

        // Getting the movement instruction from the follower.
        double[] motorPowers = follower.getCalculatedPowers();

        // Applying the motor powers.
        leftMotor.setPower(motorPowers[TankDriveVectorInterpreter.LEFT_MOTOR_INDEX_ID]);
        rightMotor.setPower(motorPowers[TankDriveVectorInterpreter.RIGHT_MOTOR_INDEX_ID]);

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
