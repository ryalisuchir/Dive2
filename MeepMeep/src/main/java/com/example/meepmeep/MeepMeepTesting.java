package com.example.meepmeep;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueLight())
                .setConstraints(60, 50, Math.toRadians(180), Math.toRadians(180), 12)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-6.70, 30.38, Math.toRadians(-90)))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-31, 27.95, Math.toRadians(0)), Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-31, 20, Math.toRadians(90)), Math.toRadians(90))
                .strafeToConstantHeading(
                        new Vector2d(-38, 20),
                        new TranslationalVelConstraint(60)
                )

                .splineToConstantHeading(new Vector2d(-45, 52), Math.toRadians(90))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-45, 20), Math.toRadians(90))

                .splineToConstantHeading(new Vector2d(-53, 17), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-53, 54), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(-27.5, 54), Math.toRadians(90))
                .strafeToLinearHeading(
                        new Vector2d(-27.5, 66), Math.toRadians(90),
                        new TranslationalVelConstraint(15)
                )

                .build());

        Image fieldBackground = null;
        try {
            fieldBackground = ImageIO.read(new File("/Users/rrrr/Juice-INTO-THE-DEEP-Black.png"));
        } catch (IOException ignored) {
        }

        meepMeep.setBackground(fieldBackground)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}