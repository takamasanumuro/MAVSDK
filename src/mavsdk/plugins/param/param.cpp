// WARNING: THIS FILE IS AUTOGENERATED! As such, it should not be edited.
// Edits need to be made to the proto files
// (see https://github.com/mavlink/MAVSDK-Proto/blob/master/protos/param/param.proto)

#include <iomanip>

#include "param_impl.h"
#include "plugins/param/param.h"

namespace mavsdk {

using IntParam = Param::IntParam;
using FloatParam = Param::FloatParam;
using CustomParam = Param::CustomParam;
using AllParams = Param::AllParams;

Param::Param(System& system) : PluginBase(), _impl{std::make_unique<ParamImpl>(system)} {}

Param::Param(std::shared_ptr<System> system) :
    PluginBase(),
    _impl{std::make_unique<ParamImpl>(system)}
{}

Param::~Param() {}

std::pair<Param::Result, int32_t> Param::get_param_int(std::string name) const
{
    return _impl->get_param_int(name);
}

Param::Result Param::set_param_int(std::string name, int32_t value) const
{
    return _impl->set_param_int(name, value);
}

std::pair<Param::Result, float> Param::get_param_float(std::string name) const
{
    return _impl->get_param_float(name);
}

Param::Result Param::set_param_float(std::string name, float value) const
{
    return _impl->set_param_float(name, value);
}

std::pair<Param::Result, std::string> Param::get_param_custom(std::string name) const
{
    return _impl->get_param_custom(name);
}

Param::Result Param::set_param_custom(std::string name, std::string value) const
{
    return _impl->set_param_custom(name, value);
}

Param::AllParams Param::get_all_params() const
{
    return _impl->get_all_params();
}

bool operator==(const Param::IntParam& lhs, const Param::IntParam& rhs)
{
    return (rhs.name == lhs.name) && (rhs.value == lhs.value);
}

std::ostream& operator<<(std::ostream& str, Param::IntParam const& int_param)
{
    str << std::setprecision(15);
    str << "int_param:" << '\n' << "{\n";
    str << "    name: " << int_param.name << '\n';
    str << "    value: " << int_param.value << '\n';
    str << '}';
    return str;
}

bool operator==(const Param::FloatParam& lhs, const Param::FloatParam& rhs)
{
    return (rhs.name == lhs.name) &&
           ((std::isnan(rhs.value) && std::isnan(lhs.value)) || rhs.value == lhs.value);
}

std::ostream& operator<<(std::ostream& str, Param::FloatParam const& float_param)
{
    str << std::setprecision(15);
    str << "float_param:" << '\n' << "{\n";
    str << "    name: " << float_param.name << '\n';
    str << "    value: " << float_param.value << '\n';
    str << '}';
    return str;
}

bool operator==(const Param::CustomParam& lhs, const Param::CustomParam& rhs)
{
    return (rhs.name == lhs.name) && (rhs.value == lhs.value);
}

std::ostream& operator<<(std::ostream& str, Param::CustomParam const& custom_param)
{
    str << std::setprecision(15);
    str << "custom_param:" << '\n' << "{\n";
    str << "    name: " << custom_param.name << '\n';
    str << "    value: " << custom_param.value << '\n';
    str << '}';
    return str;
}

bool operator==(const Param::AllParams& lhs, const Param::AllParams& rhs)
{
    return (rhs.int_params == lhs.int_params) && (rhs.float_params == lhs.float_params) &&
           (rhs.custom_params == lhs.custom_params);
}

std::ostream& operator<<(std::ostream& str, Param::AllParams const& all_params)
{
    str << std::setprecision(15);
    str << "all_params:" << '\n' << "{\n";
    str << "    int_params: [";
    for (auto it = all_params.int_params.begin(); it != all_params.int_params.end(); ++it) {
        str << *it;
        str << (it + 1 != all_params.int_params.end() ? ", " : "]\n");
    }
    str << "    float_params: [";
    for (auto it = all_params.float_params.begin(); it != all_params.float_params.end(); ++it) {
        str << *it;
        str << (it + 1 != all_params.float_params.end() ? ", " : "]\n");
    }
    str << "    custom_params: [";
    for (auto it = all_params.custom_params.begin(); it != all_params.custom_params.end(); ++it) {
        str << *it;
        str << (it + 1 != all_params.custom_params.end() ? ", " : "]\n");
    }
    str << '}';
    return str;
}

std::ostream& operator<<(std::ostream& str, Param::Result const& result)
{
    switch (result) {
        case Param::Result::Unknown:
            return str << "Unknown";
        case Param::Result::Success:
            return str << "Success";
        case Param::Result::Timeout:
            return str << "Timeout";
        case Param::Result::ConnectionError:
            return str << "Connection Error";
        case Param::Result::WrongType:
            return str << "Wrong Type";
        case Param::Result::ParamNameTooLong:
            return str << "Param Name Too Long";
        case Param::Result::NoSystem:
            return str << "No System";
        case Param::Result::ParamValueTooLong:
            return str << "Param Value Too Long";
        default:
            return str << "Unknown";
    }
}

} // namespace mavsdk