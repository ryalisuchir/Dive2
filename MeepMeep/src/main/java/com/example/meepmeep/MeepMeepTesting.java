package com.example.meepmeep;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveTrainType;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setDriveTrainType(DriveTrainType.MECANUM)
                .setColorScheme(new ColorSchemeRedDark())
                .build();

        Pose2d initialPose = new Pose2d(new Vector2d(-11.8, -61.7), Math.PI / 2);

        // Run action sequence
        try {
            myBot.runAction(randomMove(myBot, initialPose, 10));
        } catch (IllegalArgumentException e) {
            System.err.println("An error occurred while running the randomMove sequence: " + e.getMessage());
        }

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .setAxesInterval(10)
                .start();
    }

    static Action randomMove(RoadRunnerBotEntity myBot, Pose2d currentPosition, int iterations) {
        System.out.println("\nGenerating " + iterations + " random moves");
        Action[] output = new Action[iterations];

        for (int i = 0; i < iterations; i++) {
            Pose2d targetPosition = new Pose2d(
                    new Vector2d(Math.random() * 60 * 2 - 60, Math.random() * 60 * 2 - 60),
                    Math.random() * Math.PI * 2
            );

            double headingDifference = Math.toDegrees(currentPosition.heading.toDouble() - targetPosition.heading.toDouble());
            System.out.println(headingDifference);
            boolean isReversed = Math.abs(headingDifference) > 180;
            System.out.printf("Move %d: headingDifference=%.2f, isReversed=%b\n", i + 1, headingDifference, isReversed);

            try {
                switch ((int) (Math.random() * 5)) {
                    case 0:
                        output[i] = myBot.getDrive().actionBuilder(currentPosition)
                            .splineToSplineHeading(targetPosition, Math.random() * Math.PI * 2)
                            .setReversed(isReversed)
                            .build();
                        break;
                    case 1:
                        output[i] = myBot.getDrive().actionBuilder(currentPosition)
                            .splineTo(targetPosition.position, targetPosition.heading)
                            .setReversed(isReversed)
                            .build();
                        break;
                    case 2:
                        output[i] = myBot.getDrive().actionBuilder(currentPosition)
                            .strafeTo(targetPosition.position)
                            .setReversed(isReversed)
                            .build();
                        break;
                    case 3:
                        output[i] = myBot.getDrive().actionBuilder(currentPosition)
                            .splineToLinearHeading(targetPosition, Math.random() * Math.PI * 2)
                            .setReversed(isReversed)
                            .build();
                        break;
                    case 4:
                        output[i] = myBot.getDrive().actionBuilder(currentPosition)
                            .strafeToLinearHeading(targetPosition.position, targetPosition.heading)
                            .setReversed(isReversed)
                            .build();
                        break;
                }
            } catch (IllegalArgumentException e) {
                System.err.println("Failed to create action for iteration " + (i + 1) + ": " + e.getMessage());
                // Optionally, handle error or continue with next iteration
                continue;
            }

            currentPosition = targetPosition;
            System.out.println((i + 1) + " of " + iterations + " moves generated");
        }

        System.out.println("\nRunning sequence");
        return new SequentialAction(output);
    }
}
