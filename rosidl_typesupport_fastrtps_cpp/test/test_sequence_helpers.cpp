// Copyright 2026 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>
#include <vector>

#include "fastcdr/Cdr.h"
#include "fastcdr/FastBuffer.h"
#include "fastcdr/exceptions/NotEnoughMemoryException.h"

#include "rosidl_typesupport_fastrtps_cpp/serialization_helpers.hpp"

using eprosima::fastcdr::Cdr;
using eprosima::fastcdr::CdrVersion;
using eprosima::fastcdr::FastBuffer;

namespace
{

struct Shifted
{
  uint32_t a;
  uint32_t b;
  double c;
  bool operator==(const Shifted & o) const {return a == o.a && b == o.b && c == o.c;}
};

struct Padded
{
  int32_t a;
  double b;
  bool operator==(const Padded & o) const {return a == o.a && b == o.b;}
};

struct Flagged
{
  float x;
  bool f;
  uint8_t a;
  uint16_t b;
  bool operator==(const Flagged & o) const {return x == o.x && f == o.f && a == o.a && b == o.b;}
};

bool ser(const Shifted & m, Cdr & cdr) {cdr << m.a << m.b << m.c; return true;}
bool des(Cdr & cdr, Shifted & m) {cdr >> m.a >> m.b >> m.c; return true;}
bool ser(const Padded & m, Cdr & cdr) {cdr << m.a << m.b; return true;}
bool des(Cdr & cdr, Padded & m) {cdr >> m.a >> m.b; return true;}
bool ser(const Flagged & m, Cdr & cdr)
{
  cdr << m.x << static_cast<uint8_t>(m.f ? 1 : 0) << m.a << m.b;
  return true;
}
bool des(Cdr & cdr, Flagged & m)
{
  uint8_t f;
  cdr >> m.x >> f >> m.a >> m.b;
  m.f = f != 0;
  return true;
}

size_t size_of(const Shifted &, size_t a)
{
  size_t s = a;
  s += 4 + Cdr::alignment(s, 4);
  s += 4 + Cdr::alignment(s, 4);
  s += 8 + Cdr::alignment(s, 8);
  return s - a;
}

auto deserialize = [](Cdr & cdr, auto & m) {return des(cdr, m);};
auto serialize = [](const auto & m, Cdr & cdr) {return ser(m, cdr);};
auto plain = []() {return true;};

template<typename T>
std::vector<T> make(size_t n);

template<>
std::vector<Shifted> make(size_t n)
{
  std::vector<Shifted> v;
  for (size_t i = 0; i < n; ++i) {
    v.push_back({static_cast<uint32_t>(i + 1), static_cast<uint32_t>(3 * i), 0.5 * i});
  }
  return v;
}

template<>
std::vector<Padded> make(size_t n)
{
  std::vector<Padded> v;
  for (size_t i = 0; i < n; ++i) {
    v.push_back({static_cast<int32_t>(i), -0.25 * i});
  }
  return v;
}

template<>
std::vector<Flagged> make(size_t n)
{
  std::vector<Flagged> v;
  for (size_t i = 0; i < n; ++i) {
    v.push_back(
      {static_cast<float>(i), i % 2 == 0, static_cast<uint8_t>(i),
        static_cast<uint16_t>(7 * i)});
  }
  return v;
}

// Serializes `prefix` bytes, then the items one by one, as generated code did before the helpers.
template<typename T>
std::vector<char> reference_stream(
  size_t prefix, const std::vector<T> & items, Cdr::Endianness endianness)
{
  std::vector<char> buffer(64 + items.size() * (sizeof(T) + 8), 0);
  FastBuffer fb(buffer.data(), buffer.size());
  Cdr cdr(fb, endianness, CdrVersion::DDS_CDR);
  for (size_t i = 0; i < prefix; ++i) {
    cdr << static_cast<uint8_t>(i);
  }
  cdr << static_cast<uint32_t>(items.size());
  for (const auto & item : items) {
    ser(item, cdr);
  }
  buffer.resize(cdr.get_serialized_data_length());
  return buffer;
}

template<typename T>
void check_round_trip(size_t prefix, size_t n, Cdr::Endianness endianness)
{
  auto items = make<T>(n);
  auto expected = reference_stream(prefix, items, endianness);

  std::vector<char> buffer(expected.size(), 0);
  FastBuffer fb(buffer.data(), buffer.size());
  Cdr ser_cdr(fb, endianness, CdrVersion::DDS_CDR);
  for (size_t i = 0; i < prefix; ++i) {
    ser_cdr << static_cast<uint8_t>(i);
  }
  ser_cdr << static_cast<uint32_t>(items.size());
  rosidl_typesupport_fastrtps_cpp::cdr_serialize_message_array(
    ser_cdr, items.data(), items.size(), serialize, deserialize, plain);
  ASSERT_EQ(ser_cdr.get_serialized_data_length(), expected.size());
  EXPECT_EQ(0, std::memcmp(buffer.data(), expected.data(), expected.size()));

  for (size_t shift = 0; shift < 8; ++shift) {
    std::vector<char> storage(expected.size() + 8);
    std::memcpy(storage.data() + shift, expected.data(), expected.size());
    FastBuffer in(storage.data() + shift, expected.size());
    Cdr cdr(in, endianness, CdrVersion::DDS_CDR);
    uint8_t skip;
    for (size_t i = 0; i < prefix; ++i) {
      cdr >> skip;
    }
    uint32_t size;
    cdr >> size;
    std::vector<T> out = make<T>(3);
    rosidl_typesupport_fastrtps_cpp::cdr_deserialize_message_sequence(
      cdr, out, size, deserialize, plain);
    EXPECT_EQ(out, items) << "prefix " << prefix << " n " << n << " shift " << shift;
    EXPECT_EQ(cdr.get_serialized_data_length(), expected.size());
  }
}

}  // namespace

TEST(SequenceHelpers, memory_image_detection)
{
  using rosidl_typesupport_fastrtps_cpp::detail::cdr_is_memory_image;
  EXPECT_TRUE(cdr_is_memory_image<Shifted>(deserialize));
  EXPECT_FALSE(cdr_is_memory_image<Padded>(deserialize));
  EXPECT_FALSE(cdr_is_memory_image<Flagged>(deserialize));
}

TEST(SequenceHelpers, message_sequences_match_element_by_element)
{
  for (auto endianness : {Cdr::LITTLE_ENDIANNESS, Cdr::BIG_ENDIANNESS}) {
    for (size_t prefix = 0; prefix < 9; ++prefix) {
      for (size_t n : {0, 1, 2, 3, 50}) {
        check_round_trip<Shifted>(prefix, n, endianness);
        check_round_trip<Padded>(prefix, n, endianness);
        check_round_trip<Flagged>(prefix, n, endianness);
      }
    }
  }
}

TEST(SequenceHelpers, serialized_size_matches_element_by_element)
{
  for (size_t start = 0; start < 16; ++start) {
    for (size_t n : {0, 1, 2, 3, 50}) {
      auto items = make<Shifted>(n);
      size_t expected = start;
      for (const auto & item : items) {
        expected += size_of(item, expected);
      }
      EXPECT_EQ(
        expected, rosidl_typesupport_fastrtps_cpp::serialized_size_message_array(
          items.data(), items.size(), start, [](const auto & m, size_t a) {return size_of(m, a);},
          deserialize, plain));
    }
  }
}

template<typename T>
void check_primitive(size_t prefix, size_t n, Cdr::Endianness endianness)
{
  std::vector<T> items;
  for (size_t i = 0; i < n; ++i) {
    items.push_back(static_cast<T>(i * 37 + 1));
  }
  std::vector<char> buffer(64 + n * sizeof(T), 0);
  FastBuffer fb(buffer.data(), buffer.size());
  Cdr ser_cdr(fb, endianness, CdrVersion::DDS_CDR);
  for (size_t i = 0; i < prefix; ++i) {
    ser_cdr << static_cast<uint8_t>(i);
  }
  ser_cdr << items;
  ser_cdr << static_cast<uint8_t>(0x7e);
  size_t length = ser_cdr.get_serialized_data_length();

  for (size_t shift = 0; shift < 8; ++shift) {
    std::vector<char> storage(length + 8);
    std::memcpy(storage.data() + shift, buffer.data(), length);
    FastBuffer in(storage.data() + shift, length);
    Cdr cdr(in, endianness, CdrVersion::DDS_CDR);
    uint8_t skip;
    for (size_t i = 0; i < prefix; ++i) {
      cdr >> skip;
    }
    std::vector<T> out(5, static_cast<T>(9));
    rosidl_typesupport_fastrtps_cpp::cdr_deserialize_sequence(cdr, out);
    EXPECT_EQ(out, items);
    uint8_t tail = 0;
    cdr >> tail;
    EXPECT_EQ(tail, 0x7e);
  }

  if (n > 0) {
    FastBuffer in(buffer.data(), length - 2);
    Cdr cdr(in, endianness, CdrVersion::DDS_CDR);
    uint8_t skip;
    for (size_t i = 0; i < prefix; ++i) {
      cdr >> skip;
    }
    auto before = cdr.get_current_position();
    std::vector<T> out;
    EXPECT_THROW(
      rosidl_typesupport_fastrtps_cpp::cdr_deserialize_sequence(cdr, out),
      eprosima::fastcdr::exception::NotEnoughMemoryException);
    EXPECT_EQ(before, cdr.get_current_position());
  }
}

TEST(SequenceHelpers, primitive_sequences_match_fastcdr)
{
  for (auto endianness : {Cdr::LITTLE_ENDIANNESS, Cdr::BIG_ENDIANNESS}) {
    for (size_t prefix = 0; prefix < 9; ++prefix) {
      for (size_t n : {0, 1, 2, 3, 1000}) {
        check_primitive<uint8_t>(prefix, n, endianness);
        check_primitive<int8_t>(prefix, n, endianness);
        check_primitive<uint16_t>(prefix, n, endianness);
        check_primitive<float>(prefix, n, endianness);
        check_primitive<double>(prefix, n, endianness);
        check_primitive<int64_t>(prefix, n, endianness);
      }
    }
  }
}

TEST(SequenceHelpers, oversized_length_does_not_allocate)
{
  std::vector<char> buffer(16, 0);
  FastBuffer fb(buffer.data(), buffer.size());
  Cdr ser_cdr(fb, Cdr::DEFAULT_ENDIAN, CdrVersion::DDS_CDR);
  ser_cdr << static_cast<uint32_t>(0xffffffffu);
  FastBuffer in(buffer.data(), buffer.size());
  Cdr cdr(in, Cdr::DEFAULT_ENDIAN, CdrVersion::DDS_CDR);
  std::vector<double> out;
  EXPECT_THROW(
    rosidl_typesupport_fastrtps_cpp::cdr_deserialize_sequence(cdr, out),
    eprosima::fastcdr::exception::NotEnoughMemoryException);
  EXPECT_TRUE(out.empty());
}
