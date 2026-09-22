#include "json/ReJson.hpp"

#include <opencv2/core.hpp>

#include <iostream>
#include <sys/stat.h>
#include <ctime>

bool ReJson::verifyModifiedTime()
{
    struct stat file_state{};
    if (stat(this->path_.c_str(), &file_state) != 0)
    {
        std::cout << "json file path stat error." << std::endl;
        return false;
    }

    std::time_t modified_time = file_state.st_mtime;

    if (this->last_modified_time_ == 0 && this->last_modified_time_ != modified_time)
    {
        this->last_modified_time_ = modified_time;
        return false;
    }

    bool status = (this->last_modified_time_ != modified_time);
    this->last_modified_time_ = modified_time;
    return status;
}

void ReJson::updateJson()
{
    if (this->verifyModifiedTime())
    {
        cv::FileStorage new_config;
        try
        {
            new_config.open(this->path_, cv::FileStorage::READ);
        }
        catch (const cv::Exception &e)
        {
            std::cout << "json config reload failed: " << e.what() << std::endl;
            return;
        }

        if (!new_config.isOpened())
        {
            std::cout << "json config reload failed: " << this->path_ << std::endl;
            return;
        }

        config_ = std::move(new_config);
        std::cout << "json config changed." << std::endl;
    }
}
