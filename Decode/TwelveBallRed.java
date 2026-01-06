package org.firstinspires.ftc.teamcode.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Subsystems.Gate;
import org.firstinspires.ftc.teamcode.Subsystems.Hood;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelDeadlineGroup;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;

@Autonomous(name="Twelve Ball Red")
public class TwelveBallRed extends NextFTCOpMode {

    public PathChain ScorePreload;
    public PathChain IntakeFirst;
    public PathChain EmptyGate;
    public PathChain ScoreFirst;
    public PathChain IntakeSecond;
    public PathChain ScoreSecond;
    public PathChain IntakeThird;
    public PathChain ScoreThird;
    public PathChain Leave;

    Follower follower;
    public void buildPaths(Follower follower) {

        ScorePreload = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(144-26, 128), new Pose(144-56, 98))
                )
                .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-75))
                .build();

        IntakeFirst = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(144-56, 98),
                                new Pose(144-49.418, 85.400),
                                new Pose(144-44.000, 83.450),
                                new Pose(144-20.455, 83.618)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        EmptyGate = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(144-20.455, 83.618),
                                new Pose(144-22.000, 72.818),
                                new Pose(144-15, 73.470)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                .build();

        ScoreFirst = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(144-15, 73.470), new Pose(144-56, 98))
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(-75))
                .build();

        IntakeSecond = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(144-56, 98),
                                new Pose(144-53, 68),
                                new Pose(144-47, 61),
                                new Pose(144-10, 57)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        ScoreSecond = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(144-8, 57),
                                new Pose(144-45.164, 58.745),
                                new Pose(144-55.800, 67.582),
                                new Pose(144-56, 98)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        IntakeThird = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(144-56, 98),
                                new Pose(144-49.582, 43),
                                new Pose(144-42.709, 38),
                                new Pose(144-13.909, 34)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        ScoreThird = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(144-13.909, 34),
                                new Pose(144-42.709, 32.073),
                                new Pose(144-49.582, 49.255),
                                new Pose(144-56, 98)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        Leave = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(144-56, 98), new Pose(144-56, 110))
                )
                .setLinearHeadingInterpolation(Math.toRadians(-75), Math.toRadians(-130))
                .build();
    }

    Command fullAuto;

    public TwelveBallRed() {
        addComponents(
                new SubsystemComponent(Shooter.SHOOTER, Intake.INTAKE, Gate.GATE, Turret.TURRET, Hood.HOOD),
                new PedroComponent(Constants::createFollower)
        );
    }

    AutoCommands autoCommands;

    @Override
    public void onInit() {

        follower = PedroComponent.follower();
        follower.setStartingPose(new Pose(144-26, 128, Math.toRadians(-90)));
        buildPaths(follower);

        autoCommands = new AutoCommands(hardwareMap);

        // TODO: TUNE TURRET ANGLE, HOOD ANGLE, AND RPM'S FOR ALL SHOOTING SEQUENCES

        Command preload = new SequentialGroup(
                new ParallelDeadlineGroup(
                        new ParallelGroup(
                                new FollowPath(ScorePreload),
                                AutoCommands.autoThreshold(890),
                                new Delay(2)
                        ),
                        AutoCommands.initializeLauncher(890, -70, 0)
                ),
                AutoCommands.fireBalls(890)
        );

        Command firstThree = new SequentialGroup(

                // Emptying the gate after intaking the balls
                new ParallelGroup(
                        new FollowPath(IntakeFirst),
                        AutoCommands.startIntake()
                ),
                new FollowPath(EmptyGate),
                AutoCommands.stopIntake(),
                new Delay(1.0),
                // Moving to position & spinning up
                new ParallelDeadlineGroup(
                        new ParallelGroup(
                                new FollowPath(ScoreFirst),
                                AutoCommands.autoThreshold(890),
                                new Delay(1.5)
                        ),
                        AutoCommands.initializeLauncher(890, -70, 0)
                ),
                new Delay(0.5),
                // Firing balls
                AutoCommands.fireBalls(890)
        );

        Command secondThree = new SequentialGroup(
                // Intaking balls
                new ParallelGroup(
                        new FollowPath(IntakeSecond),
                        AutoCommands.startIntake()
                ),
                AutoCommands.stopIntake(),
                // Spinning up shooter and returning to shooting location
                new ParallelDeadlineGroup(
                        new ParallelGroup(
                                AutoCommands.autoThreshold(890),
                                new FollowPath(ScoreSecond),
                                new Delay(3.5)
                        ),
                        AutoCommands.stopIntake(),
                        new Delay(0.5),
                        AutoCommands.initializeLauncher(890, -55, 0)
                ),
                // Firing balls
                AutoCommands.fireBalls(890)
        );

        Command thirdThree = new SequentialGroup(
                // Intaking balls
                new ParallelGroup(
                        new FollowPath(IntakeThird),
                        AutoCommands.startIntake()
                ),
                AutoCommands.stopIntake(),
                // Spinning up shooter and returning to shooting location
                new ParallelDeadlineGroup(
                        new ParallelGroup(
                                AutoCommands.autoThreshold(890),
                                new FollowPath(ScoreThird),
                                new Delay(4.0)
                        ),
                        AutoCommands.stopIntake(),
                        AutoCommands.initializeLauncher(890, -60, 0)
                ),
                // Firing balls
                AutoCommands.fireBalls(890)
        );
        fullAuto = new SequentialGroup(
                preload,
                firstThree,
                secondThree,
                thirdThree,
                new FollowPath(Leave)
        );
    }

    @Override
    public void onStartButtonPressed() {
        fullAuto.schedule();
    }
}