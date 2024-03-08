#ifndef HANDS_ON_2_UTIL_HPP_
#define HANDS_ON_2_UTIL_HPP_

/**
 * @brief Remove all characters that might have been stored up in 
 *        the serial input buffer prior to running this program
 */
void purgeSerial();

/**
 * @brief Wait for the character 's' to be pressed before starting
 *        the program
 */
void waitForStart();

/**
 * @brief Check to see if the character 's' has been pressed to 
 *        stop the program and kill the motors 
 */
void checkForStop();

#endif
