#include "grinder_chassis_driver/emergency_record.h"

#include <json/json.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <pwd.h>
#include <unistd.h>

#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <limits.h>
#include <locale>
#include <mutex>
#include <sstream>
#include <stdexcept>

namespace grinder_chassis_driver {
namespace {
std::string systemError(const std::string& operation) {
  const int code = errno;
  return operation + ": " + std::strerror(code);
}

std::string parentPath(const std::string& path) {
  const auto slash = path.find_last_of('/');
  return slash == std::string::npos ? "." : (slash == 0 ? "/" : path.substr(0, slash));
}

bool createDirectories(const std::string& path, std::string& error) {
  if (path.empty() || path == "." || path == "/") return true;
  struct stat info;
  if (::stat(path.c_str(), &info) == 0) {
    if (S_ISDIR(info.st_mode)) return true;
    error = path + " is not a directory";
    return false;
  }
  if (errno != ENOENT) { error = systemError("stat " + path); return false; }
  if (!createDirectories(parentPath(path), error)) return false;
  if (::mkdir(path.c_str(), 0777) == 0) return true;
  if (errno == EEXIST && ::stat(path.c_str(), &info) == 0 && S_ISDIR(info.st_mode)) return true;
  error = systemError("mkdir " + path);
  return false;
}

bool atomicWrite(const std::string& path, const std::string& contents, std::string& error) {
  const std::string parent = parentPath(path), temporary = path + ".tmp";
  if (!createDirectories(parent, error)) return false;
  int fd = ::open(temporary.c_str(), O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC, 0666);
  if (fd < 0) { error = systemError("open " + temporary); return false; }
  std::size_t written = 0;
  while (written < contents.size()) {
    const ssize_t count = ::write(fd, contents.data() + written, contents.size() - written);
    if (count < 0 && errno == EINTR) continue;
    if (count <= 0) {
      error = count == 0 ? "short file write: " + temporary : systemError("write " + temporary);
      ::close(fd);
      ::unlink(temporary.c_str());
      return false;
    }
    written += static_cast<std::size_t>(count);
  }
  if (::fsync(fd) != 0) {
    error = systemError("fsync " + temporary);
    ::close(fd);
    ::unlink(temporary.c_str());
    return false;
  }
  if (::close(fd) != 0) {
    error = systemError("close " + temporary);
    ::unlink(temporary.c_str());
    return false;
  }
  if (::rename(temporary.c_str(), path.c_str()) != 0) {
    error = systemError("rename " + temporary);
    ::unlink(temporary.c_str());
    return false;
  }
  // Durably record the rename as well as the contents of the file.
  fd = ::open(parent.c_str(), O_RDONLY | O_DIRECTORY | O_CLOEXEC);
  if (fd < 0) { error = systemError("open directory " + parent); return false; }
  const int synced = ::fsync(fd);
  if (synced != 0) error = systemError("fsync directory " + parent);
  const int closed = ::close(fd);
  if (closed != 0 && synced == 0) error = systemError("close directory " + parent);
  return synced == 0 && closed == 0;
}

std::string trim(const std::string& value) {
  const auto first = value.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) return "";
  return value.substr(first, value.find_last_not_of(" \t\r\n") - first + 1);
}

bool objectDocumentBounds(const std::string& contents) {
  const auto first = contents.find_first_not_of(" \t\r\n");
  const auto last = contents.find_last_not_of(" \t\r\n");
  if (first == std::string::npos || contents[first] != '{' || contents[last] != '}') return false;
  // JSON has no comments. This is only a lexical comment guard, not a JSON
  // parser: JsonCpp below still validates strings, escapes, values and nesting.
  // A slash inside a quoted JSON string (URLs, paths) remains valid.
  bool quoted = false, escaped = false;
  for (char character : contents) {
    if (quoted) {
      if (escaped) escaped = false;
      else if (character == '\\') escaped = true;
      else if (character == '"') quoted = false;
    } else if (character == '"') quoted = true;
    else if (character == '/') return false;
  }
  return true;
}
}  // namespace

std::string expandUserPath(const std::string& input) {
  std::string path = input;
  if (!path.empty() && path[0] == '~') {
    const auto slash = path.find('/');
    const std::string username = path.substr(1, slash == std::string::npos ? slash : slash - 1);
    std::string home;
    if (username.empty()) {
      if (const char* environment_home = std::getenv("HOME")) home = environment_home;
      else if (const passwd* entry = ::getpwuid(::getuid())) home = entry->pw_dir;
    } else if (const passwd* entry = ::getpwnam(username.c_str())) home = entry->pw_dir;
    if (!home.empty()) path = home + (slash == std::string::npos ? "" : path.substr(slash));
  }
  if (path.empty() || path[0] != '/') {
    char cwd[PATH_MAX];
    if (!::getcwd(cwd, sizeof(cwd))) throw std::runtime_error(systemError("getcwd"));
    path = std::string(cwd) + (path.empty() ? "" : "/" + path);
  }
  return path;
}

bool persistManualSpeedLimit(const std::string& input, double limit, std::string& error) {
  error.clear();
  if (!std::isfinite(limit) || limit < 0.0) { error = "manual speed limit must be finite and nonnegative"; return false; }
  if (trim(input).empty()) { error = "config_file is empty"; return false; }
  const std::string path = expandUserPath(trim(input));
  std::ifstream source(path, std::ios::binary);
  if (!source) { error = "cannot read config_file: " + path; return false; }
  std::ostringstream original;
  original << source.rdbuf();
  if (source.bad()) { error = "cannot read config_file: " + path; return false; }
  const std::string text = original.str();
  std::ostringstream formatted;
  formatted.imbue(std::locale::classic());
  formatted << std::setprecision(6) << limit;
  const std::string setting = "manual_drive_max_wheel_speed: " + formatted.str();
  std::string updated;
  bool found = false;
  std::size_t start = 0;
  while (start < text.size()) {
    const auto newline = text.find('\n', start);
    const std::size_t end = newline == std::string::npos ? text.size() : newline + 1;
    const std::string line = text.substr(start, end - start);
    const auto first = line.find_first_not_of(" \t\r\n");
    if (first != std::string::npos && line.compare(first, 29, "manual_drive_max_wheel_speed:") == 0) {
      std::string ending;
      if (!line.empty() && line.back() == '\n') ending = line.size() > 1 && line[line.size() - 2] == '\r' ? "\r\n" : "\n";
      updated += line.substr(0, first) + setting + ending;
      found = true;
    } else updated += line;
    start = end;
  }
  if (!found) {
    const std::string newline = text.find("\r\n") == std::string::npos ? "\n" : "\r\n";
    if (!updated.empty() && updated.back() != '\n') updated += newline;
    updated += setting + newline;
  }
  return atomicWrite(path, updated, error);
}

struct EmergencyRecord::Impl {
  explicit Impl(std::string file) : path(std::move(file)) {}
  std::string path;
  mutable std::mutex state_mutex;
  std::mutex io_mutex;
  bool value = false;
  uint64_t revision = 0, saved_revision = 0;
};

EmergencyRecord::EmergencyRecord(const std::string& path) : impl_(new Impl(expandUserPath(path))) {
  struct stat info;
  if (::stat(impl_->path.c_str(), &info) != 0 && errno == ENOENT) return;
  std::ifstream input(impl_->path, std::ios::binary);
  Json::Value document;
  Json::CharReaderBuilder builder;
  builder["failIfExtra"] = true;
  builder["allowComments"] = false;
  std::string error;
  try {
    const std::string contents((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
    // Keep the complete record inside a parsed array. Extra tokens now occur
    // inside JSON syntax, where they cannot be ignored as a document suffix.
    // This avoids depending on version-specific root offset/failIfExtra logic.
    const std::string envelope = "[" + contents + ",true]";
    std::unique_ptr<Json::CharReader> reader(builder.newCharReader());
    const bool parsed = input && objectDocumentBounds(contents) &&
        reader->parse(envelope.data(), envelope.data() + envelope.size(), &document, &error) &&
        document.isArray() && document.size() == 2 && document[1].isBool() && document[1].asBool();
    const Json::Value& root = document[0];
    if (!parsed || !root.isObject() ||
        !root.isMember("emergency_observed") || !root["emergency_observed"].isBool()) {
      impl_->value = true;
      impl_->revision = 1;
    } else impl_->value = root["emergency_observed"].asBool();
  } catch (const std::exception&) {
    impl_->value = true;
    impl_->revision = 1;
  }
}

EmergencyRecord::~EmergencyRecord() = default;

bool EmergencyRecord::exists() const {
  struct stat info;
  if (::stat(impl_->path.c_str(), &info) == 0) return true;
  return errno != ENOENT;
}

bool EmergencyRecord::value() const {
  std::lock_guard<std::mutex> lock(impl_->state_mutex);
  return impl_->value;
}

uint64_t EmergencyRecord::revision() const {
  std::lock_guard<std::mutex> lock(impl_->state_mutex);
  return impl_->revision;
}

void EmergencyRecord::observe() {
  std::lock_guard<std::mutex> lock(impl_->state_mutex);
  impl_->value = true;
  ++impl_->revision;
}

bool EmergencyRecord::flush(std::string& error) {
  std::lock_guard<std::mutex> io_lock(impl_->io_mutex);
  error.clear();
  bool value;
  uint64_t revision;
  {
    std::lock_guard<std::mutex> lock(impl_->state_mutex);
    revision = impl_->revision;
    if (revision == impl_->saved_revision) return true;
    value = impl_->value;
  }
  if (!atomicWrite(impl_->path, std::string("{\"emergency_observed\": ") +
                   (value ? "true}" : "false}"), error)) return false;
  {
    std::lock_guard<std::mutex> lock(impl_->state_mutex);
    impl_->saved_revision = revision;
  }
  return true;
}

bool EmergencyRecord::clear(uint64_t expected_revision, std::string& error) {
  error.clear();
  uint64_t cleared_revision;
  {
    std::lock_guard<std::mutex> lock(impl_->state_mutex);
    if (impl_->revision != expected_revision) { error = "new emergency observed during clear"; return false; }
    impl_->value = false;
    cleared_revision = ++impl_->revision;
  }
  if (!flush(error)) {
    observe();
    return false;
  }
  bool success;
  {
    std::lock_guard<std::mutex> lock(impl_->state_mutex);
    success = !impl_->value && impl_->revision == cleared_revision;
  }
  if (!success) {
    error = "new emergency observed during clear";
    // A trip during the false write must remain dirty and be durable before a
    // failed reset returns whenever storage is available. Never hold state lock.
    std::string flush_error;
    if (!flush(flush_error) && !flush_error.empty()) error += "; " + flush_error;
  }
  return success;
}
}  // namespace grinder_chassis_driver
