#include "gtest/gtest.h"

#include <boost/scoped_ptr.hpp>

#include "parser.hpp"

using namespace std;
TEST(ROSType, builtin) {
  ROSType f;
  f.populate("string");

  EXPECT_EQ(string("string"), f.name);
  EXPECT_EQ(string("string"), f.base_type);
  EXPECT_EQ(string("string"), f.msg_name);
  EXPECT_EQ(string(""), f.pkg_name);
  EXPECT_FALSE(f.is_array);
  EXPECT_TRUE(f.is_qualified);
  EXPECT_FALSE(f.is_array);
  EXPECT_EQ(1, f.array_size);
  EXPECT_EQ(-1, f.type_size);
}

TEST(ROSType, unqualified_array) {
  ROSType f;
  f.populate("foo[40]");

  EXPECT_EQ(string("foo[40]"), f.name);
  EXPECT_EQ(string("foo"), f.base_type);
  EXPECT_EQ(string("foo"), f.msg_name);
  EXPECT_EQ(string(""), f.pkg_name);
  EXPECT_TRUE(f.is_array);
  EXPECT_FALSE(f.is_qualified);
  EXPECT_TRUE(f.is_array);
  EXPECT_EQ(40, f.array_size);
}

TEST(ROSType, builtin_fixedlen_array) {
  ROSType f;
  f.populate("float64[32]");

  EXPECT_EQ(string("float64[32]"), f.name);
  EXPECT_EQ(string("float64"), f.base_type);
  EXPECT_EQ(string("float64"), f.msg_name);
  EXPECT_EQ(string(""), f.pkg_name);
  EXPECT_TRUE(f.is_array);
  EXPECT_TRUE(f.is_qualified);
  EXPECT_TRUE(f.is_array);
  EXPECT_EQ(32, f.array_size);
  EXPECT_EQ(8, f.type_size);
}


TEST(ROSType, qualified_array) {
  ROSType f;
  f.populate("geometry_msgs/Pose[]");

  EXPECT_EQ(string("geometry_msgs/Pose[]"), f.name);
  EXPECT_EQ(string("geometry_msgs/Pose"), f.base_type);
  EXPECT_EQ(string("Pose"), f.msg_name);
  EXPECT_EQ(string("geometry_msgs"), f.pkg_name);
  EXPECT_TRUE(f.is_array);
  EXPECT_TRUE(f.is_qualified);
  EXPECT_TRUE(f.is_array);
  EXPECT_EQ(-1, f.array_size);
}

TEST(ROSTypeMap, bad_def) {
  ROSTypeMap rtm;
  string def("foo field1\n"
             "==\n"
             "MSG: asdf/foo\n"
             "uint8 field2\n"
             "==\n"
             "MSG: qwerty/foo\n"
             "uint8 field3\n"
             "\n");
  ASSERT_THROW(rtm.populate(def), invalid_argument);
}

TEST(ROSTypeMap, no_embedded_defs) {
  ROSTypeMap rtm;
  string def("Header header\n"
             "\n"
             "geometry_msgs/Quaternion orientation\n"
             "float64[9] orientation_covariance\n"
             "\n"
             "geometry_msgs/Vector3 angular_velocity\n"
             "float64[9] angular_velocity_covariance\n"
             "\n"
             "geometry_msgs/Vector3 linear_acceleration\n"
             "float64[9] linear_acceleration_covariance\n");
  ASSERT_THROW(rtm.populate(def), invalid_argument);
}

TEST(ROSTypeMap, multiple_embedded_defs) {
  ROSTypeMap rtm;
  string def("Header header\n"
             "==\n"
             "MSG: std_msgs/Header\n"
             "uint8 field\n"
             "==\n"
             "MSG: std_msgs/Header\n"
             "uint8 field\n");
  ASSERT_THROW(rtm.populate(def), invalid_argument);
}

TEST(ROSMessageFields, parse_quaternion_def) {
  ROSMessageFields mt;
  string
    def("MSG: geometry_msgs/Quaternion\n"
        "# This represents an orientation in free space in quaternion form.\n"
        "\n"
        "float64 x\n"
        "float64 y\n"
        "float64 z\n"
        "float64 w\n");
  mt.populate(def);
  EXPECT_EQ(string("geometry_msgs/Quaternion"), mt.type().name);
  ASSERT_EQ(4, mt.nfields());
  EXPECT_EQ(string("float64"), mt.at(0).type.name);
  EXPECT_EQ(string("float64"), mt.at(1).type.name);
  EXPECT_EQ(string("float64"), mt.at(2).type.name);
  EXPECT_EQ(string("float64"), mt.at(3).type.name);
  EXPECT_EQ(string("x"), mt.at(0).name);
  EXPECT_EQ(string("y"), mt.at(1).name);
  EXPECT_EQ(string("z"), mt.at(2).name);
  EXPECT_EQ(string("w"), mt.at(3).name);
  EXPECT_FALSE(mt.at(0).constant);
  EXPECT_FALSE(mt.at(1).constant);
  EXPECT_FALSE(mt.at(2).constant);
  EXPECT_FALSE(mt.at(3).constant);
}

TEST(ROSMessageFields, parse_comments) {
  ROSMessageFields mt;
  string
    def("MSG: geometry_msgs/Quaternion\n"
        "\n"
        "          # I'm a comment after whitespace\n"
        "float64 x # I'm an end of line comment float64 y\n"
        "float64 z\n"
        );
  mt.populate(def);
  EXPECT_EQ(string("geometry_msgs/Quaternion"), mt.type().name);
  ASSERT_EQ(2, mt.nfields());
  EXPECT_EQ(string("float64"), mt.at(0).type.name);
  EXPECT_EQ(string("float64"), mt.at(1).type.name);
  EXPECT_EQ(string("x"), mt.at(0).name);
  EXPECT_EQ(string("z"), mt.at(1).name);
  EXPECT_FALSE(mt.at(0).constant);
  EXPECT_FALSE(mt.at(1).constant);
}

TEST(ROSMessageFields, constant_uint8) {
  ROSMessageFields fields;
  string def("uint8 a = 1\n");
  fields.populate(def);
  ASSERT_EQ(1, fields.nfields());
  EXPECT_EQ(string("a"), fields.at(0).name);
  EXPECT_EQ(string("uint8"), fields.at(0).type.base_type);
  EXPECT_TRUE(fields.at(0).constant);
  EXPECT_EQ(string("1"), fields.at(0).value);
  EXPECT_EQ(1, fields.at(0).bytes.size());
  EXPECT_EQ(1, fields.at(0).bytes[0]);
}

TEST(ROSMessageFields, constant_string) {
  ROSMessageFields fields;
  string def("string msg = ab9\n");
  fields.populate(def);
  ASSERT_EQ(1, fields.nfields());
  EXPECT_EQ(string("msg"), fields.at(0).name);
  EXPECT_EQ(string("string"), fields.at(0).type.base_type);
  EXPECT_TRUE(fields.at(0).constant);
  EXPECT_EQ(string("ab9"), fields.at(0).value);

  uint8_t bytes[] = {'a', 'b', '9'};
  EXPECT_EQ(7, fields.at(0).bytes.size());
  EXPECT_EQ(3, fields.at(0).bytes[0]);
  EXPECT_EQ(0, fields.at(0).bytes[1]);
  EXPECT_EQ(0, fields.at(0).bytes[2]);
  EXPECT_EQ(0, fields.at(0).bytes[3]);
  EXPECT_EQ(bytes[0], fields.at(0).bytes[4]);
  EXPECT_EQ(bytes[1], fields.at(0).bytes[5]);
  EXPECT_EQ(bytes[2], fields.at(0).bytes[6]);
}

TEST(ROSMessageFields, constant_comments) {
  ROSMessageFields fields;
  string def(
"string str=  this string has a # comment in it  \n"
"string str2 = this string has \"quotes\" and \\slashes\\ in it\n"
"float64 a=64.0 # numeric comment\n");
  fields.populate(def);
  ASSERT_EQ(3, fields.nfields());
  EXPECT_EQ(string("str"), fields.at(0).name);
  EXPECT_EQ(string("string"), fields.at(0).type.base_type);
  EXPECT_TRUE(fields.at(0).constant);
  EXPECT_EQ(string("this string has a # comment in it"), fields.at(0).value);

  EXPECT_EQ(string("str2"), fields.at(1).name);
  EXPECT_EQ(string("string"), fields.at(1).type.base_type);
  EXPECT_TRUE(fields.at(1).constant);
  EXPECT_EQ(string("this string has \"quotes\" and \\slashes\\ in it"),
            fields.at(1).value);

  EXPECT_EQ(string("a"), fields.at(2).name);
  EXPECT_EQ(string("float64"), fields.at(2).type.base_type);
  EXPECT_TRUE(fields.at(2).constant);
  EXPECT_EQ(string("64.0"), fields.at(2).value);
}

TEST(ROSTypeMap, parse_pose_def) {
  string
    def(
"# A representation of pose in free space, composed of postion and orientation. \n"
"Point position\n"
"Quaternion orientation\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Point\n"
"# This contains the position of a point in free space\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Quaternion\n"
"# This represents an orientation in free space in quaternion form.\n"
"\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"float64 w");

  ROSTypeMap rtm;
  rtm.populate(def);

  const ROSMessageFields *type;
  ASSERT_TRUE((type = rtm.getMsgFields(string("0-root"))) != NULL);
  EXPECT_EQ(2, type->nfields());
  EXPECT_EQ(string("position"), type->at(0).name);
  EXPECT_EQ(string("geometry_msgs/Point"), type->at(0).type.base_type);

  ASSERT_TRUE((type = rtm.getMsgFields(string("geometry_msgs/Quaternion"))) != NULL);
  EXPECT_EQ(4, type->nfields());

  ASSERT_TRUE((type = rtm.getMsgFields(string("geometry_msgs/Point"))) != NULL);
  EXPECT_EQ(3, type->nfields());
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
