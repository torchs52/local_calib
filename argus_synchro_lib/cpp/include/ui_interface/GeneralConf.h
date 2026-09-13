#pragma once

#include <string>

struct GeneralConf
{
  public:
    bool in_factory;
    int operation_mode;
    bool has_external_guard;
    double external_guard_offset;
    double ground_height;
    double ground_height_margin;
    double rotation_radius;
    std::string initial_transform_file;

    GeneralConf(bool in_factory, int operation_mode, bool has_external_guard, double external_guard_offset,
                double ground_height, double ground_height_margin, double rotation_radius,
                const std::string& initial_transform_file)
        : in_factory(in_factory), operation_mode(operation_mode), has_external_guard(has_external_guard),
          external_guard_offset(external_guard_offset), ground_height(ground_height),
          ground_height_margin(ground_height_margin), rotation_radius(rotation_radius),
          initial_transform_file(initial_transform_file)
    {
    }
};