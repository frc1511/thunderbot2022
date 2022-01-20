#pragma once
// stolen from 2020
class Feedback {
public:
    void sendString(const char* subsystem, const char* name, const char* format, ...);
    void sendDouble(const char* subsystem, const char* name, double value);
    void sendBoolean(const char* subsystem, const char* name, bool yesno);

    void sendEditableDouble(const char* subsystem, const char* name, double value);
    double getEditableDouble(const char* subsystem, const char* name, double fallback);
};