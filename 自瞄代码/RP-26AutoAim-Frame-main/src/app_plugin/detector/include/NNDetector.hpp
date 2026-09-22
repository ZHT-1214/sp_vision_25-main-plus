#pragma once

#include "Detector.hpp"

class NNDetector final : public Detector
{
public:
    void process(const app::Context &context) override;
};
