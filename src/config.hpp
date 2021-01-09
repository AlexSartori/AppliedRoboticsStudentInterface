#ifndef CONFIG_MANAGER_H
#define CONFIG_MANAGER_H

#include "../libs/configuru/configuru.hpp"

namespace student {
    configuru::Config openConfigFile();

    float readFloatConfig(const std::string key);

    int readIntConfig(const std::string key);

    std::string readStringConfig(const std::string key);

    bool readBooleanConfig(const std::string key);
}

#endif //CONFIG_MANAGER_H
