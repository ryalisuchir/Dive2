package com.example.meepmeep;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
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
                .setConstraints(60, 80, Math.toRadians(180), Math.toRadians(180), 12)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(0, -62, Math.toRadians(-90)))
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(30,-40),Math.toRadians(-90),new TranslationalVelConstraint(200))
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(50,-10),Math.toRadians(0))
                .setTangent(Math.toRadians(-70))

                .splineToConstantHeading(new Vector2d(50, -45), Math.toRadians(-90))
                // done push #1
                .setTangent(Math.toRadians(135))
                .splineToConstantHeading(new Vector2d(58,-10),Math.toRadians(-90))

                .setTangent(Math.toRadians(-70))
                .splineToConstantHeading(new Vector2d(58,-48),Math.toRadians(-90),new TranslationalVelConstraint(250))

                .setTangent(Math.toRadians(135))
                .splineToConstantHeading(new Vector2d(65,-10),Math.toRadians(-90))

                .setTangent(Math.toRadians(-70))
                .splineToConstantHeading(new Vector2d(65,-48),Math.toRadians(-90),new TranslationalVelConstraint(250))

                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(40,-61),Math.toRadians(-90))
                .strafeTo(new Vector2d(0,-28),new TranslationalVelConstraint(150))
                .splineToConstantHeading(new Vector2d(40,-61),Math.toRadians(-90))
                .strafeToConstantHeading(new Vector2d(-1,-28),new TranslationalVelConstraint(150))

                .splineToConstantHeading(new Vector2d(40,-61),Math.toRadians(-90))
                .strafeToConstantHeading(new Vector2d(-2,-28),new TranslationalVelConstraint(150))

                .splineToConstantHeading(new Vector2d(40,-61),Math.toRadians(-90))
                .strafeToConstantHeading(new Vector2d(-3,-28),new TranslationalVelConstraint(150))

                .splineToConstantHeading(new Vector2d(40,-61),Math.toRadians(-90))
                .strafeToConstantHeading(new Vector2d(-4,-28),new TranslationalVelConstraint(150))


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