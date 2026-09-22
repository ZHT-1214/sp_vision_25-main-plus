#pragma once

#include "FireControlSystem.hpp"

class PlannerControl final : public FireControlSystem
{
public:
    void process(const app::Context &context) override;
};
