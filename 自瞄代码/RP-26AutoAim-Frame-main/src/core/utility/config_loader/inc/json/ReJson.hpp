#pragma once

#include <ctime>
#include <string>

#include <opencv2/core/persistence.hpp>

class ReJson {
public:
    template<typename T>
    ReJson(const T &path) : path_(path) {
        config_.open(path_, cv::FileStorage::READ);
    }
    ~ReJson() = default;

    cv::FileStorage config_;
    void updateJson();

private:
    std::string path_;
    time_t last_modified_time_ = 0;
    bool verifyModifiedTime();
};
