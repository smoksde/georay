#pragma once

// A minimal JSON reader, for QGroundControl .plan mission files.
//
// Why not a library: the subset needed here is a handful of numbers out of a
// known, self-authored file, and pulling in a general-purpose JSON header to
// read them would be the largest dependency in a project that currently has
// three. The grammar is small enough to implement correctly, and the tests in
// selftest.cpp exercise it harder than .plan files will.
//
// It is nonetheless a parser reading a file from disk, so it is written to
// fail rather than misbehave on bad input: depth and length are bounded,
// every conversion is checked, and malformed input returns an error instead
// of a partially-built value.

#include <map>
#include <memory>
#include <string>
#include <vector>

namespace georay {

class JsonValue {
public:
    enum class Type { Null, Bool, Number, String, Array, Object };

    JsonValue() = default;

    Type type() const { return type_; }
    bool isNull() const { return type_ == Type::Null; }
    bool isNumber() const { return type_ == Type::Number; }
    bool isString() const { return type_ == Type::String; }
    bool isArray() const { return type_ == Type::Array; }
    bool isObject() const { return type_ == Type::Object; }

    // Accessors that never throw. A missing key, a wrong type or an
    // out-of-range index all yield the fallback, because a mission file with
    // an unexpected shape should produce a clear "this is not a mission"
    // rather than an exception from three layers down.
    double number(double fallback = 0.0) const {
        return type_ == Type::Number ? number_ : fallback;
    }
    bool boolean(bool fallback = false) const { return type_ == Type::Bool ? bool_ : fallback; }
    const std::string& string(const std::string& fallback = kEmptyString) const {
        return type_ == Type::String ? string_ : fallback;
    }

    size_t size() const {
        if (type_ == Type::Array) return array_.size();
        if (type_ == Type::Object) return object_.size();
        return 0;
    }
    // Index into an array; out of range gives a null value.
    const JsonValue& at(size_t index) const {
        return (type_ == Type::Array && index < array_.size()) ? array_[index] : kNull;
    }
    // Look up an object member; missing gives a null value.
    const JsonValue& operator[](const std::string& key) const {
        if (type_ != Type::Object) return kNull;
        const auto found = object_.find(key);
        return found == object_.end() ? kNull : found->second;
    }
    bool has(const std::string& key) const {
        return type_ == Type::Object && object_.count(key) > 0;
    }

    // Parses a whole document. Returns false and fills `error` on anything
    // malformed, including trailing content after the top-level value.
    static bool parse(const std::string& text, JsonValue& out, std::string& error);
    static bool parseFile(const std::string& path, JsonValue& out, std::string& error);

private:
    friend class JsonParser;
    static const JsonValue kNull;
    static const std::string kEmptyString;

    Type type_ = Type::Null;
    bool bool_ = false;
    double number_ = 0.0;
    std::string string_;
    std::vector<JsonValue> array_;
    std::map<std::string, JsonValue> object_;
};

}  // namespace georay
