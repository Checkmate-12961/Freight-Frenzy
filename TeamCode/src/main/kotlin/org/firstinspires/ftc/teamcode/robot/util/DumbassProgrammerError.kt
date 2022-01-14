package org.firstinspires.ftc.teamcode.robot.util

/**
 * Thrown when the programmer is a total dumbass.
 *
 * @param thisIsWhatTheyDid Passed directly to [Error] as the message parameter.
 */
class DumbassProgrammerError(thisIsWhatTheyDid: String): Error(thisIsWhatTheyDid)