#include "geo/json.hpp"

#include <cctype>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <sstream>

namespace georay {

const JsonValue JsonValue::kNull;
const std::string JsonValue::kEmptyString;

namespace {

// Bounds chosen to be far beyond any real mission file and far below
// anything that could exhaust memory or the stack. A .plan with a thousand
// waypoints is about 300 KB and four levels deep.
constexpr size_t kMaxDepth = 64;
constexpr size_t kMaxLength = 64u * 1024u * 1024u;

}  // namespace

// Named and in namespace georay rather than tucked into an anonymous one, so
// that JsonValue's `friend class JsonParser` can actually refer to it: a
// class with internal linkage cannot be befriended from a header.
class JsonParser {
public:
    JsonParser(const std::string& text) : text_(text) {}

    bool parseDocument(JsonValue& out, std::string& error) {
        skipWhitespace();
        if (!parseValue(out, 0)) {
            error = error_.empty() ? "malformed JSON" : error_;
            return false;
        }
        skipWhitespace();
        if (position_ != text_.size()) {
            // Trailing content means the file is not what it claims to be;
            // accepting it would mean silently reading half a document.
            error = "unexpected content after the top-level value at byte " +
                    std::to_string(position_);
            return false;
        }
        return true;
    }

private:
    bool fail(const std::string& what) {
        if (error_.empty()) error_ = what + " at byte " + std::to_string(position_);
        return false;
    }

    void skipWhitespace() {
        while (position_ < text_.size()) {
            const char c = text_[position_];
            if (c == ' ' || c == '\t' || c == '\n' || c == '\r') {
                ++position_;
            } else {
                break;
            }
        }
    }

    bool literal(const char* word) {
        const size_t length = std::char_traits<char>::length(word);
        if (text_.compare(position_, length, word) != 0) return false;
        position_ += length;
        return true;
    }

    bool parseValue(JsonValue& out, size_t depth) {
        if (depth > kMaxDepth) return fail("nesting too deep");
        if (position_ >= text_.size()) return fail("unexpected end of input");

        switch (text_[position_]) {
            case '{': return parseObject(out, depth);
            case '[': return parseArray(out, depth);
            case '"': {
                std::string value;
                if (!parseString(value)) return false;
                out.type_ = JsonValue::Type::String;
                out.string_ = std::move(value);
                return true;
            }
            case 't':
                if (!literal("true")) return fail("expected 'true'");
                out.type_ = JsonValue::Type::Bool;
                out.bool_ = true;
                return true;
            case 'f':
                if (!literal("false")) return fail("expected 'false'");
                out.type_ = JsonValue::Type::Bool;
                out.bool_ = false;
                return true;
            case 'n':
                if (!literal("null")) return fail("expected 'null'");
                out.type_ = JsonValue::Type::Null;
                return true;
            default: return parseNumber(out);
        }
    }

    bool parseNumber(JsonValue& out) {
        const size_t start = position_;
        if (position_ < text_.size() && text_[position_] == '-') ++position_;

        // JSON forbids a leading zero followed by more digits, and forbids a
        // number with no integer part. Being strict here is what stops "01"
        // or "-" from being read as 1 and 0.
        if (position_ >= text_.size() || !std::isdigit(uc(text_[position_])))
            return fail("expected a digit");
        if (text_[position_] == '0') {
            ++position_;
        } else {
            while (position_ < text_.size() && std::isdigit(uc(text_[position_]))) ++position_;
        }

        if (position_ < text_.size() && text_[position_] == '.') {
            ++position_;
            if (position_ >= text_.size() || !std::isdigit(uc(text_[position_])))
                return fail("expected a digit after the decimal point");
            while (position_ < text_.size() && std::isdigit(uc(text_[position_]))) ++position_;
        }
        if (position_ < text_.size() && (text_[position_] == 'e' || text_[position_] == 'E')) {
            ++position_;
            if (position_ < text_.size() && (text_[position_] == '+' || text_[position_] == '-'))
                ++position_;
            if (position_ >= text_.size() || !std::isdigit(uc(text_[position_])))
                return fail("expected a digit in the exponent");
            while (position_ < text_.size() && std::isdigit(uc(text_[position_]))) ++position_;
        }

        const std::string token = text_.substr(start, position_ - start);
        // strtod rather than stod: no exceptions, and the end pointer
        // confirms the whole token was consumed.
        char* end = nullptr;
        const double value = std::strtod(token.c_str(), &end);
        if (end != token.c_str() + token.size() || !std::isfinite(value))
            return fail("not a finite number");

        out.type_ = JsonValue::Type::Number;
        out.number_ = value;
        return true;
    }

    bool parseString(std::string& out) {
        if (position_ >= text_.size() || text_[position_] != '"') return fail("expected a string");
        ++position_;
        out.clear();
        while (true) {
            if (position_ >= text_.size()) return fail("unterminated string");
            const char c = text_[position_++];
            if (c == '"') return true;
            if (c != '\\') {
                // Control characters are not legal unescaped in JSON.
                if (uc(c) < 0x20) return fail("control character in string");
                out.push_back(c);
                continue;
            }
            if (position_ >= text_.size()) return fail("unterminated escape");
            const char escape = text_[position_++];
            switch (escape) {
                case '"': out.push_back('"'); break;
                case '\\': out.push_back('\\'); break;
                case '/': out.push_back('/'); break;
                case 'b': out.push_back('\b'); break;
                case 'f': out.push_back('\f'); break;
                case 'n': out.push_back('\n'); break;
                case 'r': out.push_back('\r'); break;
                case 't': out.push_back('\t'); break;
                case 'u': {
                    if (position_ + 4 > text_.size()) return fail("truncated \\u escape");
                    unsigned int code = 0;
                    for (int i = 0; i < 4; ++i) {
                        const char h = text_[position_ + size_t(i)];
                        code <<= 4;
                        if (h >= '0' && h <= '9') code |= unsigned(h - '0');
                        else if (h >= 'a' && h <= 'f') code |= unsigned(h - 'a' + 10);
                        else if (h >= 'A' && h <= 'F') code |= unsigned(h - 'A' + 10);
                        else return fail("bad hex digit in \\u escape");
                    }
                    position_ += 4;
                    // Encoded as UTF-8. Surrogate pairs are not recombined:
                    // nothing in a mission file needs astral-plane characters,
                    // and each half is emitted as-is rather than guessed at.
                    appendUtf8(out, code);
                    break;
                }
                default: return fail("unknown escape");
            }
        }
    }

    static void appendUtf8(std::string& out, unsigned int code) {
        if (code < 0x80) {
            out.push_back(char(code));
        } else if (code < 0x800) {
            out.push_back(char(0xC0 | (code >> 6)));
            out.push_back(char(0x80 | (code & 0x3F)));
        } else {
            out.push_back(char(0xE0 | (code >> 12)));
            out.push_back(char(0x80 | ((code >> 6) & 0x3F)));
            out.push_back(char(0x80 | (code & 0x3F)));
        }
    }

    bool parseArray(JsonValue& out, size_t depth) {
        ++position_;  // '['
        out.type_ = JsonValue::Type::Array;
        skipWhitespace();
        if (position_ < text_.size() && text_[position_] == ']') {
            ++position_;
            return true;
        }
        while (true) {
            JsonValue element;
            skipWhitespace();
            if (!parseValue(element, depth + 1)) return false;
            out.array_.push_back(std::move(element));
            skipWhitespace();
            if (position_ >= text_.size()) return fail("unterminated array");
            if (text_[position_] == ',') {
                ++position_;
                continue;
            }
            if (text_[position_] == ']') {
                ++position_;
                return true;
            }
            return fail("expected ',' or ']'");
        }
    }

    bool parseObject(JsonValue& out, size_t depth) {
        ++position_;  // '{'
        out.type_ = JsonValue::Type::Object;
        skipWhitespace();
        if (position_ < text_.size() && text_[position_] == '}') {
            ++position_;
            return true;
        }
        while (true) {
            skipWhitespace();
            std::string key;
            if (!parseString(key)) return false;
            skipWhitespace();
            if (position_ >= text_.size() || text_[position_] != ':') return fail("expected ':'");
            ++position_;
            skipWhitespace();
            JsonValue value;
            if (!parseValue(value, depth + 1)) return false;
            out.object_[key] = std::move(value);
            skipWhitespace();
            if (position_ >= text_.size()) return fail("unterminated object");
            if (text_[position_] == ',') {
                ++position_;
                continue;
            }
            if (text_[position_] == '}') {
                ++position_;
                return true;
            }
            return fail("expected ',' or '}'");
        }
    }

    static unsigned char uc(char c) { return static_cast<unsigned char>(c); }

    const std::string& text_;
    size_t position_ = 0;
    std::string error_;
};

bool JsonValue::parse(const std::string& text, JsonValue& out, std::string& error) {
    if (text.size() > kMaxLength) {
        error = "document is implausibly large";
        return false;
    }
    JsonParser parser(text);
    JsonValue parsed;
    if (!parser.parseDocument(parsed, error)) return false;
    out = std::move(parsed);
    return true;
}

bool JsonValue::parseFile(const std::string& path, JsonValue& out, std::string& error) {
    std::ifstream file(path, std::ios::binary);
    if (!file) {
        error = "cannot open " + path;
        return false;
    }
    std::ostringstream buffer;
    buffer << file.rdbuf();
    if (!JsonValue::parse(buffer.str(), out, error)) {
        error = path + ": " + error;
        return false;
    }
    return true;
}

}  // namespace georay
