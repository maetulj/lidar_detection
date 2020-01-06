#pragma once

#include "external/json.hpp"
#include <fstream>
#include <pcl/common/common.h>

class Options
{
    using json = nlohmann::json;

public:
    /**
     * Construct the object and load the config file.
     */
    Options()
    {
        this->load();
    }

    /**
     * Load the specified JSON file.
     *
     * @param file <std::string> JSON file to load.
     */
    void load(const std::string file = "../src/config.json")
    {
        std::ifstream config_file(file, std::ios::in);

        if (!config_file.is_open())
        {
            throw std::runtime_error("JSON File not found!");
        }

        m_options = nlohmann::json::parse(config_file);
    }

    nlohmann::json const& operator()() const
    {
        return this->m_options;
    }

    Eigen::Vector4f region(const std::string type) const
    {
        return Eigen::Vector4f(
            m_options["filtering"][type]["x"],
            m_options["filtering"][type]["y"],
            m_options["filtering"][type]["z"], 1
        );
    }

private:
    nlohmann::json m_options;
};