// Implementation of the alarm signal handler and timer logic (M4)

#include <stdio.h>
#include <unistd.h>
#include <signal.h>
#include <stdlib.h>
#include <errno.h>

// Global state variables
static int g_nRetransmissions = 0;
static int g_alarmEnabled = 0;
static int g_timeoutSec = 0;

// Flag to indicate if an alarm was received (and retransmission is needed)
static volatile int g_alarm = 0;

/**
 * @brief Signal handler for SIGALRM. Sets the global flag to signal retransmission.
 * @param signum The signal number (SIGALRM).
 */
void alarmHandler(int signum)
{
    if (signum == SIGALRM) {
        g_nRetransmissions++;
        g_alarm = 1;
        printf("\nAlarm #%d received: retransmitting SET\n", g_nRetransmissions);
    }
}

/**
 * @brief Sets up the SIGALRM handler. Must be called before enabling the timer.
 */
void setupAlarmHandler()
{
    struct sigaction sa;

    // Clear the struct and set the handler
    sigemptyset(&sa.sa_mask);
    sa.sa_handler = alarmHandler;

    if (sigaction(SIGALRM, &sa, NULL) == -1) {
        perror("Error setting up SIGALRM handler");
        exit(1);
    }
}

/**
 * @brief Starts the retransmission timer.
 * @param timeoutSec The timeout value in seconds.
 * @param maxTries The maximum number of retransmission attempts (only used for context here).
 */
void enableAlarm(int timeoutSec, int maxTries)
{
    g_timeoutSec = timeoutSec;
    g_alarmEnabled = 1;
    g_nRetransmissions = 0; // Reset counter for a new connection attempt
    g_alarm = 0;            // Reset alarm flag

    // Start the timer
    alarm(g_timeoutSec);
}

/**
 * @brief Disables the retransmission timer.
 */
void disableAlarm()
{
    alarm(0); // Cancel any pending alarm
    g_alarmEnabled = 0;
    g_alarm = 0;
}

/**
 * @brief Checks if the alarm flag is set.
 * @return 1 if alarm received, 0 otherwise.
 */
int isAlarmSet() {
    return g_alarm;
}

/**
 * @brief Clears the alarm flag after retransmission.
 */
void clearAlarm() {
    g_alarm = 0;
}

/**
 * @brief Returns the current number of retransmission attempts.
 */
int getRetransmissionCount() {
    return g_nRetransmissions;
}
