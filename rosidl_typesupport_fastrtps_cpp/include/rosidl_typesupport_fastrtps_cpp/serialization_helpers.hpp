// Copyright 2024 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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

#ifndef ROSIDL_TYPESUPPORT_FASTRTPS_CPP__SERIALIZATION_HELPERS_HPP_
#define ROSIDL_TYPESUPPORT_FASTRTPS_CPP__SERIALIZATION_HELPERS_HPP_

#include <rosidl_typesupport_fastrtps_cpp/visibility_control.h>

#include <fastcdr/Cdr.h>
#include <fastcdr/FastBuffer.h>
#include <fastcdr/exceptions/BadParamException.h>
#include <fastcdr/exceptions/NotEnoughMemoryException.h>

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <iterator>
#include <limits>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <vector>

namespace rosidl_typesupport_fastrtps_cpp
{

inline void cdr_serialize(
  eprosima::fastcdr::Cdr & cdr,
  const std::u16string & u16str)
{
  if (u16str.size() > std::numeric_limits<uint32_t>::max()) {
    throw std::overflow_error("String length exceeds does not fit in CDR serialization format");
  }
  auto len = static_cast<uint32_t>(u16str.size());
  cdr << len;
  for (uint32_t i = 0; i < len; ++i) {
    // We are serializing a uint32_t for interoperability with other DDS-based implementations.
    // We might change this to a uint16_t in the future if we don't mind breaking backward
    // compatibility and we make the change for all the supported DDS implementations at the same
    // time.
    uint32_t c = static_cast<uint32_t>(u16str[i]);
    cdr << c;
  }
}

inline bool cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  std::u16string & u16str)
{
  uint32_t len;
  cdr >> len;
  try {
    u16str.resize(len);
  } catch (...) {
    return false;
  }
  for (uint32_t i = 0; i < len; ++i) {
    // We are serializing a uint32_t for interoperability with other DDS-based implementations.
    // If we change this to a uint16_t in the future, we could remove the check below, since all
    // serialized values would fit in the destination type.
    uint32_t c;
    cdr >> c;
    if (c > std::numeric_limits<std::u16string::value_type>::max()) {
      throw eprosima::fastcdr::exception::BadParamException(
              "Character value exceeds maximum value");
    }
    u16str[i] = static_cast<std::u16string::value_type>(c);
  }

  return true;
}

namespace detail
{

template<typename T>
class unaligned_iterator
{
public:
  using iterator_category = std::random_access_iterator_tag;
  using value_type = T;
  using difference_type = std::ptrdiff_t;
  using pointer = const T *;
  using reference = T;

  explicit unaligned_iterator(const char * position)
  : position_(position) {}

  T operator*() const
  {
    T value;
    std::memcpy(static_cast<void *>(&value), position_, sizeof(T));
    return value;
  }
  T operator[](difference_type n) const {return *(*this + n);}
  unaligned_iterator & operator++() {position_ += sizeof(T); return *this;}
  unaligned_iterator operator++(int) {auto old = *this; ++*this; return old;}
  unaligned_iterator & operator--() {position_ -= sizeof(T); return *this;}
  unaligned_iterator operator--(int) {auto old = *this; --*this; return old;}
  unaligned_iterator & operator+=(difference_type n) {position_ += n * sizeof(T); return *this;}
  unaligned_iterator & operator-=(difference_type n) {position_ -= n * sizeof(T); return *this;}
  unaligned_iterator operator+(difference_type n) const {return unaligned_iterator(*this) += n;}
  unaligned_iterator operator-(difference_type n) const {return unaligned_iterator(*this) -= n;}
  difference_type operator-(const unaligned_iterator & other) const
  {
    return (position_ - other.position_) / static_cast<difference_type>(sizeof(T));
  }
  bool operator==(const unaligned_iterator & other) const {return position_ == other.position_;}
  bool operator!=(const unaligned_iterator & other) const {return position_ != other.position_;}
  bool operator<(const unaligned_iterator & other) const {return position_ < other.position_;}
  bool operator>(const unaligned_iterator & other) const {return position_ > other.position_;}
  bool operator<=(const unaligned_iterator & other) const {return position_ <= other.position_;}
  bool operator>=(const unaligned_iterator & other) const {return position_ >= other.position_;}

private:
  const char * position_;
};

// Builds `sequence` as {first, the next `rest` elements of the stream} with a single copy, where
// resize() would first value-initialize every element.
template<typename T, typename Allocator>
void assign_from_stream(
  eprosima::fastcdr::Cdr & cdr, std::vector<T, Allocator> & sequence, const T & first, size_t rest)
{
  const char * source = cdr.get_current_position();
  if (rest > std::numeric_limits<size_t>::max() / sizeof(T) || !cdr.jump(rest * sizeof(T))) {
    throw eprosima::fastcdr::exception::NotEnoughMemoryException(
            eprosima::fastcdr::exception::NotEnoughMemoryException::
            NOT_ENOUGH_MEMORY_MESSAGE_DEFAULT);
  }
  sequence.clear();
  sequence.reserve(rest + 1);
  sequence.push_back(first);
  if (reinterpret_cast<std::uintptr_t>(source) % alignof(T) == 0) {
    auto begin = reinterpret_cast<const T *>(source);
    sequence.insert(sequence.end(), begin, begin + rest);
  } else {
    sequence.insert(
      sequence.end(), unaligned_iterator<T>(source),
      unaligned_iterator<T>(source + rest * sizeof(T)));
  }
}

inline bool plain_stream(const eprosima::fastcdr::Cdr & cdr)
{
  return cdr.endianness() == eprosima::fastcdr::Cdr::DEFAULT_ENDIAN &&
         cdr.get_cdr_version() == eprosima::fastcdr::CdrVersion::DDS_CDR &&
         cdr.get_encoding_flag() == eprosima::fastcdr::EncodingAlgorithmFlag::PLAIN_CDR;
}

template<typename U>
void step(eprosima::fastcdr::Cdr & cdr, std::true_type)
{
  cdr << static_cast<U>(0);
}

template<typename U>
void step(eprosima::fastcdr::Cdr & cdr, std::false_type)
{
  U value;
  cdr >> value;
}

// The stream origin is not exposed, so alignment is observed by moving over one value of that size.
template<bool write>
bool at_alignment(eprosima::fastcdr::Cdr & cdr, size_t alignment)
{
  if (alignment == 1) {
    return true;
  }
  if (alignment != 2 && alignment != 4 && alignment != 8) {
    return false;
  }
  auto state = cdr.get_state();
  const char * before = cdr.get_current_position();
  bool aligned = false;
  try {
    std::integral_constant<bool, write> mode;
    if (alignment == 2) {
      step<uint16_t>(cdr, mode);
    } else if (alignment == 4) {
      step<uint32_t>(cdr, mode);
    } else {
      step<uint64_t>(cdr, mode);
    }
    aligned = static_cast<size_t>(cdr.get_current_position() - before) == alignment;
  } catch (const eprosima::fastcdr::exception::Exception &) {
  }
  cdr.set_state(state);
  return aligned;
}

// True when a T deserialized from a stream position aligned to alignof(T) is a byte copy of the
// stream: no bool, char16, string or padding, and the same member order and offsets.
template<typename T, typename Deserialize>
bool cdr_is_memory_image(Deserialize deserialize)
{
  if constexpr (!std::is_trivially_copyable<T>::value || alignof(T) > 8) {
    return false;
  } else {
    alignas(8) char image[sizeof(T)];
    for (size_t i = 0; i < sizeof(T); ++i) {
      uint32_t x = static_cast<uint32_t>(i) * 2654435761u;
      x ^= x >> 15;
      image[i] = static_cast<char>(2 + x % 254);
    }
    eprosima::fastcdr::FastBuffer buffer(image, sizeof(T));
    eprosima::fastcdr::Cdr cdr(
      buffer, eprosima::fastcdr::Cdr::DEFAULT_ENDIAN, eprosima::fastcdr::CdrVersion::DDS_CDR);
    T value;
    std::memset(static_cast<void *>(&value), 0, sizeof(T));
    try {
      if (!deserialize(cdr, value)) {
        return false;
      }
    } catch (const eprosima::fastcdr::exception::Exception &) {
      return false;
    }
    return cdr.get_serialized_data_length() == sizeof(T) &&
           std::memcmp(static_cast<const void *>(&value), image, sizeof(T)) == 0;
  }
}

}  // namespace detail

// Same result as `cdr >> sequence` for a primitive element type.
template<typename T, typename Allocator>
void cdr_deserialize_sequence(
  eprosima::fastcdr::Cdr & cdr, std::vector<T, Allocator> & sequence)
{
  static_assert(
    std::is_arithmetic<T>::value && !std::is_same<T, bool>::value && sizeof(T) <= 8,
    "primitive sequences only");
  auto state = cdr.get_state();
  uint32_t size = 0;
  cdr >> size;
  if (size == 0) {
    sequence.clear();
    return;
  }
  try {
    if (sizeof(T) > 1 && cdr.endianness() != eprosima::fastcdr::Cdr::DEFAULT_ENDIAN) {
      auto before_data = cdr.get_state();
      bool enough = cdr.jump(size);
      cdr.set_state(before_data);
      if (!enough) {
        throw eprosima::fastcdr::exception::NotEnoughMemoryException(
                eprosima::fastcdr::exception::NotEnoughMemoryException::
                NOT_ENOUGH_MEMORY_MESSAGE_DEFAULT);
      }
      sequence.resize(size);
      cdr.deserialize_array(sequence.data(), size);
      return;
    }
    T first;
    cdr >> first;
    detail::assign_from_stream(cdr, sequence, first, size - 1);
  } catch (const eprosima::fastcdr::exception::Exception &) {
    cdr.set_state(state);
    throw;
  }
}

// The message helpers below take the element (de)serializers and `plain`, which returns the
// is_plain result of the element's max_serialized_size. Elements are copied as one block when
// they are a byte image of the stream; otherwise every element goes through its own function.

// Same result as deserializing `items[0..size)` one by one.
template<typename T, typename Deserialize, typename Plain>
void cdr_deserialize_message_array(
  eprosima::fastcdr::Cdr & cdr, T * items, size_t size, Deserialize deserialize, Plain plain)
{
  static const bool memory_image = plain() && detail::cdr_is_memory_image<T>(deserialize);
  size_t i = 0;
  if (memory_image && size > 1 && detail::plain_stream(cdr)) {
    deserialize(cdr, items[0]);
    i = 1;
    if (detail::at_alignment<false>(cdr, alignof(T))) {
      const char * source = cdr.get_current_position();
      if (!cdr.jump((size - 1) * sizeof(T))) {
        throw eprosima::fastcdr::exception::NotEnoughMemoryException(
                eprosima::fastcdr::exception::NotEnoughMemoryException::
                NOT_ENOUGH_MEMORY_MESSAGE_DEFAULT);
      }
      std::memcpy(static_cast<void *>(items + 1), source, (size - 1) * sizeof(T));
      return;
    }
  }
  for (; i < size; ++i) {
    deserialize(cdr, items[i]);
  }
}

// Same result as resize(size) followed by deserializing every element.
template<typename T, typename Allocator, typename Deserialize, typename Plain>
void cdr_deserialize_message_sequence(
  eprosima::fastcdr::Cdr & cdr, std::vector<T, Allocator> & sequence, size_t size,
  Deserialize deserialize, Plain plain)
{
  static const bool memory_image = plain() && detail::cdr_is_memory_image<T>(deserialize);
  if (memory_image && size > 1 && detail::plain_stream(cdr)) {
    T first;
    deserialize(cdr, first);
    if (detail::at_alignment<false>(cdr, alignof(T))) {
      detail::assign_from_stream(cdr, sequence, first, size - 1);
      return;
    }
    sequence.resize(size);
    sequence[0] = first;
    for (size_t i = 1; i < size; ++i) {
      deserialize(cdr, sequence[i]);
    }
    return;
  }
  sequence.resize(size);
  for (size_t i = 0; i < size; ++i) {
    deserialize(cdr, sequence[i]);
  }
}

template<typename T, typename Serialize, typename Deserialize, typename Plain>
void cdr_serialize_message_array(
  eprosima::fastcdr::Cdr & cdr, const T * items, size_t size, Serialize serialize,
  Deserialize deserialize, Plain plain)
{
  static const bool memory_image = plain() && detail::cdr_is_memory_image<T>(deserialize);
  size_t i = 0;
  if (memory_image && size > 1 && detail::plain_stream(cdr)) {
    serialize(items[0], cdr);
    i = 1;
    if (detail::at_alignment<true>(cdr, alignof(T))) {
      cdr.serialize_array(reinterpret_cast<const char *>(items + 1), (size - 1) * sizeof(T));
      return;
    }
  }
  for (; i < size; ++i) {
    serialize(items[i], cdr);
  }
}

template<typename T, typename SerializedSize, typename Deserialize, typename Plain>
size_t serialized_size_message_array(
  const T * items, size_t size, size_t current_alignment, SerializedSize serialized_size,
  Deserialize deserialize, Plain plain)
{
  static const bool memory_image = plain() && detail::cdr_is_memory_image<T>(deserialize);
  size_t i = 0;
  if (memory_image && size > 1) {
    current_alignment += serialized_size(items[0], current_alignment);
    i = 1;
    if (current_alignment % alignof(T) == 0) {
      return current_alignment + (size - 1) * sizeof(T);
    }
  }
  for (; i < size; ++i) {
    current_alignment += serialized_size(items[i], current_alignment);
  }
  return current_alignment;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#endif  // ROSIDL_TYPESUPPORT_FASTRTPS_CPP__SERIALIZATION_HELPERS_HPP_
