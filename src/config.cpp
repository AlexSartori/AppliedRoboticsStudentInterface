#include "config.hpp"

namespace student {

    /*!
     * Open the configuration file
     * @return Config handler
     */
    configuru::Config openConfigFile() {
        const char *env_root = std::getenv("AR_ROOT");
        std::string baseFolder(env_root);
        configuru::Config cfg = configuru::parse_file(baseFolder + "/config.json", configuru::JSON);
        return cfg;
    }

    /*!
     * Read a configuration parameter of type float
     * @param key The key of the parameter
     * @return The float value
     */
    float readFloatConfig(const std::string key) {
        configuru::Config cfg = openConfigFile();
        return (float) cfg[key];
    }

    /*!
     * Read a configuration parameter of type int
     * @param key The key of the parameter
     * @return The int value
     */
    int readIntConfig(const std::string key) {
        configuru::Config cfg = openConfigFile();
        return (int) cfg[key];
    }

    /*!
     * Read a configuration parameter of type string
     * @param key The key of the parameter
     * @return The string value
     */
    std::string readStringConfig(const std::string key) {
        configuru::Config cfg = openConfigFile();
        return (std::string)cfg[key];
    }

    /*!
     * Read a configuration parameter of type boolean
     * @param key The key of the parameter
     * @return The boolean value
     */
    bool readBooleanConfig(const std::string key) {
        configuru::Config cfg = openConfigFile();
        return (bool)cfg[key];
    }
}
