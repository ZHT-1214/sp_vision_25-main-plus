#pragma once

namespace app
{

class Context;

class PluginBase
{
public:
    virtual ~PluginBase() = default;

    virtual void declare(Context &context) = 0;
    virtual void process(const Context &context) = 0;
};

}// namespace app
