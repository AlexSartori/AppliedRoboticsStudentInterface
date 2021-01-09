#include "config.hpp"

namespace student {
    configuru::Config openConfigFile() {
        const char *env_root = std::getenv("AR_ROOT");
        std::string baseFolder(env_root);
        configuru::Config cfg = configuru::parse_file(baseFolder + "/config.json", configuru::JSON);
        return cfg;
    }

    float readFloatConfig(const std::string key) {
        configuru::Config cfg = openConfigFile();
        return (float) cfg[key];
    }

    int readIntConfig(const std::string key) {
        configuru::Config cfg = openConfigFile();
        return (int) cfg[key];
    }

    std::string readStringConfig(const std::string key) {
        configuru::Config cfg = openConfigFile();
        return (std::string)cfg[key];
    }

    bool readBooleanConfig(const std::string key) {
        configuru::Config cfg = openConfigFile();
        return (bool)cfg[key];
    }
}
