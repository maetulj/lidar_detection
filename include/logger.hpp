#pragma once

#include <iostream>
#include <sstream>
#include <chrono>
#include <string>

#define PRINT_STREAM(args) \
    do \
    { \
        std::stringstream __logger_print_stream__; \
        __logger_print_stream__ << args; \
        std::cout << __logger_print_stream__.str() << std::endl; \
    } while (0)

/**
 * A class that logs the time and creates output.
 */
class Logger
{
public:
    /**
     * Constructs the logger object.
     * 
     * @param prefix <std::string> Prefix on output.
     */
    Logger(const std::string prefix = "", const std::string postfix = " milliseconds.")
    : m_start_time(std::chrono::steady_clock::now())
    , m_prefix(prefix)
    , m_postfix(postfix)
    {

    }

    /**
     * Destructs the object and outputs elapsed time to the standard output.
     */
    ~Logger()
    {
        const auto end_time = std::chrono::steady_clock::now();
        const auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - m_start_time);

        std::cout << m_prefix << elapsed_time.count() << m_postfix << std::endl;
    }

    /**
     * Output the number of points to the standard output.
     * 
     * @param point_number <int> Number of points to output.
     * @param prefix <std::string> Prefix for the output.
     */
    static void pointsNumber(const int point_number, const std::string prefix = "Cloud has ", const std::string postfix = " points.")
    {
        std::cout << prefix << point_number << postfix << std::endl;
    }

private:
    std::chrono::time_point<std::chrono::steady_clock> m_start_time;
    const std::string m_prefix;
    const std::string m_postfix;
};