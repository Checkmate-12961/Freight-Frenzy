/*
The MIT License (MIT)

Copyright © 2021 Checkmate Robotics

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the “Software”), to deal in the Software without restriction, including
without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the
following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT
LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN
NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

package org.firstinspires.ftc.teamcode.robot.subsystems.drivetrain.trajectorysequence;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;

/**
 * I have the best trajectory sequence runners.
 * I have to say a lot of people have been asking this question.
 * No, really. A lot of people come up to me and they ask me.
 * They say, "What's the best Trajectory Sequence Runner? And I tell them look, we know what the best Trajectory Sequence Runner is.
 * We've had almost eight years of the worst kind of runners you can imagine.
 * Oh my god, I can't believe it. Adding to and removing from the internal arrays and catching errors and so on.
 * Its terrible. Its just terrible. Look, if you want to know what the best trajectory sequence runner is, do you want to know what the best trajectory sequence runner is?
 * I'll tell you. First of all the language java, by the way I love the language java.
 * It's probably my favorite language, no it is my favorite language.
 * You know what, it's probably more like the language java but with a lot of other JVM languages.
 * A lot. If I'm being honest, I mean, if I'm being honest. I like a lot of JVM languages.
 * Except for Marco Rubio, now he's a JVM language that I don't like. Though, I probably shouldn't say that.
 * He's a nice guy but he's like, "((())())(())))((())))))))))", on and on, like that. He's like lisp! You know what I mean?
 * He's like lisp. I don't know. I mean, you know. So, we have all these internal arrays and we can add to them and subtract from them and add to them.
 * RUN THEIR CONTENTS even. Did you know that?
 * We can set them OR extend them, they don't tell you that, and I'll tell you, no one is better at array operations than me.
 * You wouldn't believe it. That I can tell you. So, we're gonna be the best on Trajectory Sequence Runners, believe me. OK? Alright.
 * You know it, I know it, we all know it, I have the best Trajectory Sequence Runners.
 * These people, they keep calling me on the phone, they say "Riley, you have the best trajectory sequence runners."
 * And I tell them, yes I know, but you know, I am a very humble guy, maybe the most humble guy, with really medium hands though.
 * And these trajectory sequence runners I have, well, they are the best trajectory sequence runners. They are going to be HUGE.
 * We are going to make trajectory sequence runners GREAT again.
 * If you listen to the FAKE NEWS guy Noah, he says, you know, that we already have good enough trajectory sequence runners, but he is a liar, and, frankly, a loser.
 * ok, ok, thank you, ok, goodnight, ok.
 *
 * Parody of AMCE Robotics "SuperArrayList"
 * See https://github.com/acmerobotics/rover-ruckus/blob/master/src/com/acmerobotics/roverruckus/util/SuperArrayList.java
 * All opinions are my own, but in an alternate universe where Noah sucks.
 */
public class SuperTrajectorySequenceRunner extends TrajectorySequenceRunner{
    public SuperTrajectorySequenceRunner(TrajectoryFollower follower, PIDCoefficients headingPIDCoefficients) {
        super(follower, headingPIDCoefficients);
    }

    public void cancelSequence() {
        super.currentTrajectorySequence = null;
    }
}
