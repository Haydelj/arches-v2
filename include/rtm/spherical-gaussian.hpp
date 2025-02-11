#pragma once

#include "int.hpp"
#include "float.hpp"
#include "vec3.hpp"
#include "quaternion.hpp"
#include "mat3.hpp"
#include "bvh.hpp"

#ifndef __riscv
#include <string>
#include <vector>
#include <fstream>
#include <map>
#include <unordered_map>
#endif

namespace rtm {

struct alignas(64) SphericalHarmonic
{
    vec3 sh[16];
};

constexpr float MIN_OPACITY = 0.01f;

struct CompressedSphericalGaussian
{

};

struct alignas(64) SphericalGaussian
{
    vec3 position;
    vec3 scale;
    quaternion rotation;
    float opacity;
    //float bounding_scale;

    BVH2::BuildObject build_object(float min_opacity = MIN_OPACITY)
    {
        float bounding_scale = max(0.0f, sqrt(-2.0f * log((min_opacity / opacity))));

        BVH2::BuildObject obj;
        obj.cost = 1.0;

        if(bounding_scale <= 0.0f)
        {
            obj.aabb.add(0.0);
            return obj;
        }

        mat3 m = mat3(rotation) * mat3(scale * bounding_scale);
        vec3 q_x = m * normalize(transpose(m) * vec3(1.0, 0.0, 0.0));
        vec3 q_y = m * normalize(transpose(m) * vec3(0.0, 1.0, 0.0));
        vec3 q_z = m * normalize(transpose(m) * vec3(0.0, 0.0, 1.0));

        obj.aabb.add(position + q_x);
        obj.aabb.add(position - q_x);

        obj.aabb.add(position + q_y);
        obj.aabb.add(position - q_y);

        obj.aabb.add(position + q_z);
        obj.aabb.add(position - q_z);

        return obj;
    }


};

#ifndef __riscv
struct PLY
{
    struct Header
    {
        struct Element
        {
            struct Property
            {
                enum class Type
                {
                    INT,
                    FLOAT,
                };

                Type type;
                uint offset;

                uint size()
                {
                    return 4;
                }
            };

            uint count;
            uint offset;
            uint size;
            std::map<std::string, Property> properties;
        };

        std::map<std::string, Element> elements;
    };

    Header header;
    std::vector<uint8_t> data;
    bool valid;

    static std::string peel(std::string& line)
    {
        uint split = std::min(line.find(' '), line.size());
        std::string first = line.substr(0, split);
        if(split == line.size()) line = "";
        else line = line.substr(split + 1);
        return first;
    }

    PLY(const std::string& file_path)
    {
        printf("Loading Ply: %s\n", file_path.c_str());

        valid = false;
        std::ifstream file_stream(file_path, std::ios::binary);
        if(file_stream.is_open())
        {
            std::string element_name;
            uint element_offset = 0;
            while(true)
            {
                char buffer[1024];
                file_stream.getline(buffer, 1024);

                std::string line(buffer);
                std::string first = peel(line);
                if(std::strcmp(first.c_str(), "end_header") == 0)
                {
                    break;
                }
                else if(std::strcmp(first.c_str(), "element") == 0)
                {
                    std::string name = peel(line);
                    header.elements[name].count = std::stoi(line);
                    header.elements[name].offset = 0;
                    element_name = name;
                }
                else if(std::strcmp(first.c_str(), "property") == 0)
                {
                    std::string type = peel(line);
                    std::string name = peel(line);
                    header.elements[element_name].properties[name].offset = header.elements[element_name].size;
                    header.elements[element_name].size += header.elements[element_name].properties[name].size();
                }
                else
                {
                    //skip
                }
            }

            size_t data_start = file_stream.tellg();
            file_stream.seekg(0, std::ios::end);
            size_t data_end = file_stream.tellg();

            data.resize(data_end - data_start);
            file_stream.seekg(data_start, std::ios::beg);
            file_stream.read((char*)data.data(), data.size());

            file_stream.close();

            valid = true;
        }

        if(!valid) printf("Failed to load Ply: %s\n", file_path.c_str());
        else       printf("Loaded Ply: %s\n", file_path.c_str());
    }

    bool read(std::vector<SphericalGaussian>& spherical_gaussians, std::vector<SphericalHarmonic>& spherical_harmonics)
    {
        if(valid)
        {
            spherical_gaussians.resize(header.elements["vertex"].count);
            spherical_harmonics.resize(header.elements["vertex"].count);

            size_t base = header.elements["vertex"].offset;
            size_t stride = header.elements["vertex"].size;

            size_t position_offset = header.elements["vertex"].properties["x"].offset;
            size_t scale_offset = header.elements["vertex"].properties["scale_0"].offset;
            size_t rotation_offset = header.elements["vertex"].properties["rot_0"].offset;
            size_t normal_offset = header.elements["vertex"].properties["nx"].offset;
            size_t opacity_offset = header.elements["vertex"].properties["opacity"].offset;
            size_t sh_offset = header.elements["vertex"].properties["f_dc_0"].offset;

            for(uint i = 0; i < spherical_gaussians.size(); ++i)
            {
                uint8_t* element = data.data() + base + i * stride;
                std::memcpy(&spherical_gaussians[i].position, element + position_offset, sizeof(vec3));
                std::memcpy(&spherical_gaussians[i].scale, element + scale_offset, sizeof(vec3));
                std::memcpy(&spherical_gaussians[i].rotation, element + rotation_offset, sizeof(quaternion));
                std::memcpy(&spherical_gaussians[i].opacity, element + opacity_offset, sizeof(float));
                std::memcpy(spherical_harmonics[i].sh, element + sh_offset, sizeof(vec3) * 16);

                //apply activations
                spherical_gaussians[i].opacity = sigmoid(spherical_gaussians[i].opacity);
                spherical_gaussians[i].scale = vec3(exp(spherical_gaussians[i].scale.x), exp(spherical_gaussians[i].scale.y), exp(spherical_gaussians[i].scale.z));

                quaternion q = spherical_gaussians[i].rotation;
                spherical_gaussians[i].rotation = normalize(quaternion(q[1], q[2], q[3], q[0]));
            }
        }

        return valid;
    }
};

#endif

}
