/*
 * SPDX-FileCopyrightText: 2006-2021 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

// This is an automatically generated file.

// Generated from the following "SharedData" msg definition:
//   float64[] content// Instances of this class can be read and written with YARP ports,
// using a ROS-compatible format.

#ifndef YARP_ROSMSG_SharedData_h
#define YARP_ROSMSG_SharedData_h

#include <yarp/os/Wire.h>
#include <yarp/os/Type.h>
#include <yarp/os/idl/WireTypes.h>
#include <string>
#include <vector>

namespace yarp {
namespace rosmsg {

class SharedData : public yarp::os::idl::WirePortable
{
public:
    std::vector<yarp::conf::float64_t> content;

    SharedData() :
            content()
    {
    }

    void clear()
    {
        // *** content ***
        content.clear();
    }

    bool readBare(yarp::os::ConnectionReader& connection) override
    {
        // *** content ***
        int len = connection.expectInt32();
        content.resize(len);
        if (len > 0 && !connection.expectBlock((char*)&content[0], sizeof(yarp::conf::float64_t)*len)) {
            return false;
        }

        return !connection.isError();
    }

    bool readBottle(yarp::os::ConnectionReader& connection) override
    {
        connection.convertTextMode();
        yarp::os::idl::WireReader reader(connection);
        if (!reader.readListHeader(1)) {
            return false;
        }

        // *** content ***
        if (connection.expectInt32() != (BOTTLE_TAG_LIST|BOTTLE_TAG_FLOAT64)) {
            return false;
        }
        int len = connection.expectInt32();
        content.resize(len);
        for (int i=0; i<len; i++) {
            content[i] = (yarp::conf::float64_t)connection.expectFloat64();
        }

        return !connection.isError();
    }

    using yarp::os::idl::WirePortable::read;
    bool read(yarp::os::ConnectionReader& connection) override
    {
        return (connection.isBareMode() ? readBare(connection)
                                        : readBottle(connection));
    }

    bool writeBare(yarp::os::ConnectionWriter& connection) const override
    {
        // *** content ***
        connection.appendInt32(content.size());
        if (content.size()>0) {
            connection.appendExternalBlock((char*)&content[0], sizeof(yarp::conf::float64_t)*content.size());
        }

        return !connection.isError();
    }

    bool writeBottle(yarp::os::ConnectionWriter& connection) const override
    {
        connection.appendInt32(BOTTLE_TAG_LIST);
        connection.appendInt32(1);

        // *** content ***
        connection.appendInt32(BOTTLE_TAG_LIST|BOTTLE_TAG_FLOAT64);
        connection.appendInt32(content.size());
        for (size_t i=0; i<content.size(); i++) {
            connection.appendFloat64(content[i]);
        }

        connection.convertTextMode();
        return !connection.isError();
    }

    using yarp::os::idl::WirePortable::write;
    bool write(yarp::os::ConnectionWriter& connection) const override
    {
        return (connection.isBareMode() ? writeBare(connection)
                                        : writeBottle(connection));
    }

    // This class will serialize ROS style or YARP style depending on protocol.
    // If you need to force a serialization style, use one of these classes:
    typedef yarp::os::idl::BareStyle<yarp::rosmsg::SharedData> rosStyle;
    typedef yarp::os::idl::BottleStyle<yarp::rosmsg::SharedData> bottleStyle;

    // The name for this message, ROS will need this
    static constexpr const char* typeName = "SharedData";

    // The checksum for this message, ROS will need this
    static constexpr const char* typeChecksum = "e3af295beedbf0b32a22c858cc47f1d3";

    // The source text for this message, ROS will need this
    static constexpr const char* typeText = "\
float64[] content\n\
";

    yarp::os::Type getType() const override
    {
        yarp::os::Type typ = yarp::os::Type::byName(typeName, typeName);
        typ.addProperty("md5sum", yarp::os::Value(typeChecksum));
        typ.addProperty("message_definition", yarp::os::Value(typeText));
        return typ;
    }
};

} // namespace rosmsg
} // namespace yarp

#endif // YARP_ROSMSG_SharedData_h
