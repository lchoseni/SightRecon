#include "srecon/config.h"
#include "srecon/euroc_dataset.h"

int main() {
    std::string config_file = "/home/yeren/OneDrive/C C++ Projects/Simple-SLAM/config/euroc_config.yaml";

    srecon::Config::SetConfigFile(config_file);
    srecon::EurocDataset::Ptr dataset = srecon::EurocDataset::Ptr(new srecon::EurocDataset());
    return 0;
}