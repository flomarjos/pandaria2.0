/**
 * @file yaml_parser.hpp
 * @author Benjamin Bogenberger
 * @brief Use yamlReadMember(...) to parse data from yaml (e.g. from ros parameter server)
 * @version 0.1
 * @date 2022-05-03
 *
 * @copyright Copyright (c) 2022
 *
 */
#pragma once

#include "XmlRpcValue.h"
#include <Eigen/Dense>
#include <iostream>
#include <sstream>
#include <string>

namespace pandaria_ros
{
namespace yaml_parser
{

template <typename T> bool parseMember(const XmlRpc::XmlRpcValue current_member, T *out){};

template <> inline bool parseMember<double>(const XmlRpc::XmlRpcValue current_member, double *out)
{
    if (current_member.getType() == XmlRpc::XmlRpcValue::TypeDouble)
    {
        *out = (double)current_member;
        return true;
    }
    else if (current_member.getType() == XmlRpc::XmlRpcValue::TypeInt)
    {
        *out = static_cast<double>((int)current_member);
        return true;
    }
    return false;
}

template <> inline bool parseMember<int>(const XmlRpc::XmlRpcValue current_member, int *out)
{
    if (current_member.getType() == XmlRpc::XmlRpcValue::TypeInt)
    {
        *out = (int)current_member;
        return true;
    }
    return false;
}

template <> inline bool parseMember<bool>(const XmlRpc::XmlRpcValue current_member, bool *out)
{
    if (current_member.getType() == XmlRpc::XmlRpcValue::TypeBoolean)
    {
        *out = (bool)current_member;
        return true;
    }
    return false;
}

template <> inline bool parseMember<Eigen::VectorXd>(const XmlRpc::XmlRpcValue current_member, Eigen::VectorXd *out)
{
    if (current_member.getType() == XmlRpc::XmlRpcValue::TypeArray)
    {
        size_t size = current_member.size();
        Eigen::VectorXd tmp_out(size);
        for (size_t i = 0; i < size; ++i)
        {
            if (!parseMember<double>(current_member[i], tmp_out.data() + i))
                return false;
        }
        *out = tmp_out;
        return true;
    }
    return false;
}

template <> inline bool parseMember<XmlRpc::XmlRpcValue>(const XmlRpc::XmlRpcValue current_member, XmlRpc::XmlRpcValue *out)
{
    *out = current_member;
    return true;
}

template <typename T>
bool depthSearch(const XmlRpc::XmlRpcValue &current_member, const std::vector<std::string>::iterator member_split_first,
                 const std::vector<std::string>::iterator member_split_end, T *out)
{

    if (std::distance(member_split_first, member_split_end) > 0)
    {
        std::string search_string = *member_split_first;

        if (current_member.getType() == XmlRpc::XmlRpcValue::TypeStruct && current_member.hasMember(search_string))
        {
            return depthSearch<T>(current_member[search_string], std::next(member_split_first), member_split_end, out);
        }
        else if (current_member.getType() == XmlRpc::XmlRpcValue::TypeArray)
        {
            for (size_t j = 0; j < current_member.size(); ++j)
            {
                if (current_member[j].getType() == XmlRpc::XmlRpcValue::TypeStruct && current_member[j].hasMember(search_string))
                {
                    return depthSearch<T>(current_member[j][search_string], std::next(member_split_first), member_split_end, out);
                }
            }
        }
    }
    else
    {
        T tmp_out;
        if (parseMember<T>(current_member, &tmp_out))
        {
            *out = tmp_out;
            return true;
        }
    }
    return false;
}

template <typename T> bool yamlReadMember(const XmlRpc::XmlRpcValue &yaml, const std::string &member, T *out)
{
    std::stringstream stream(member);
    std::string segment;
    std::vector<std::string> member_split;
    while (std::getline(stream, segment, '/'))
    {
        member_split.push_back(segment);
    }

    return depthSearch<T>(yaml, member_split.begin(), member_split.end(), out);
}

}  // namespace yaml_parser
}  // namespace pandaria_ros