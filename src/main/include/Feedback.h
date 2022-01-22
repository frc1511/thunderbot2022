#pragma once

// stolen from 2020 by Trevor Wiesen and edited by Peter :D

/**
 * The feedback class handles sending and getting values from the dashboard.
 */
class Feedback {
public:
    /**
     * Sends a string value to the dashboard.
     */
    static void sendString(const char* subsystem, const char* name, const char* format, ...);

    /**
     * Sends a double value to the dashboard.
     */
    static void sendDouble(const char* subsystem, const char* name, double value);

    /**
     * Sends a boolean value to the dashboard.
     */
    static void sendBoolean(const char* subsystem, const char* name, bool yesno);

    /**
     * Sends an editable double value to the dashboard.
     */
    static void sendEditableDouble(const char* subsystem, const char* name, double value);

    /**
     * Gets the value of an editable double from the dashboard.
     */
    static double getEditableDouble(const char* subsystem, const char* name, double fallback);
};