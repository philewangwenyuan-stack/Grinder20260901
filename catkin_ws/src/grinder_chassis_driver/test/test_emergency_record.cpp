#include "grinder_chassis_driver/emergency_record.h"

#include <gtest/gtest.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <dirent.h>

#include <atomic>
#include <chrono>
#include <fstream>
#include <future>
#include <limits>
#include <sstream>
#include <thread>

namespace grinder_chassis_driver {
namespace {
void removeTree(const std::string& path) {
  struct stat info;
  if (::lstat(path.c_str(), &info) != 0) return;
  if (!S_ISDIR(info.st_mode)) { ::unlink(path.c_str()); return; }
  if (DIR* directory = ::opendir(path.c_str())) {
    while (dirent* entry = ::readdir(directory)) {
      const std::string name = entry->d_name;
      if (name != "." && name != "..") removeTree(path + "/" + name);
    }
    ::closedir(directory);
  }
  ::rmdir(path.c_str());
}

class RecordTest : public ::testing::Test {
 protected:
  void SetUp() override {
    char pattern[] = "/tmp/grinder-record-XXXXXX";
    const char* created = ::mkdtemp(pattern);
    ASSERT_NE(nullptr, created);
    directory = created;
    path = directory + "/emergency.json";
  }
  void TearDown() override { if (!directory.empty()) removeTree(directory); }
  void write(const std::string& contents, const std::string& file = "") {
    std::ofstream target(file.empty() ? path : file, std::ios::binary);
    target << contents;
    ASSERT_TRUE(target.good());
  }
  std::string read(const std::string& file) {
    std::ifstream source(file, std::ios::binary);
    std::ostringstream result;
    result << source.rdbuf();
    return result.str();
  }
  std::string directory, path;
};

TEST_F(RecordTest, MissingRecordDoesNotLatchAndObservationIsDeferredUntilFlush) {
  EmergencyRecord record(path);
  EXPECT_FALSE(record.exists());
  EXPECT_FALSE(record.value());
  EXPECT_EQ(0u, record.revision());
  record.observe();
  EXPECT_TRUE(record.value());
  EXPECT_FALSE(record.exists());
  std::string error;
  ASSERT_TRUE(record.flush(error)) << error;
  EXPECT_TRUE(record.exists());
  EXPECT_TRUE(EmergencyRecord(path).value());
  EXPECT_EQ("{\"emergency_observed\": true}", read(path));
  EXPECT_EQ(-1, ::access((path + ".tmp").c_str(), F_OK));
  ASSERT_TRUE(record.clear(record.revision(), error)) << error;
  EXPECT_FALSE(record.value());
  EXPECT_FALSE(EmergencyRecord(path).value());
}

TEST_F(RecordTest, PythonRecordAndLegacyDisplayFormatsStayCompatible) {
  write("{\"emergency_observed\": true}");
  EmergencyRecord legacy(path);
  EXPECT_TRUE(legacy.value());
  const std::string motion_path = directory + "/motion.json";
  EmergencyRecord motion(motion_path);
  EXPECT_FALSE(motion.exists());
  // The controller owns first-upgrade migration; this demonstrates its record contract.
  if (!motion.exists() && legacy.value()) motion.observe();
  std::string error;
  ASSERT_TRUE(motion.flush(error)) << error;
  ASSERT_TRUE(motion.clear(motion.revision(), error)) << error;
  EmergencyRecord restarted(motion_path);
  EXPECT_TRUE(restarted.exists());
  EXPECT_FALSE(restarted.value());
  EXPECT_TRUE(legacy.value());  // A display record cannot relatch an existing motion record.
  write("{\"extra\": 2, \"emergency_observed\": false}");
  EXPECT_FALSE(EmergencyRecord(path).value());
  write(" \n{\"url\": \"https://example.test/a/*b*/\", \"escaped\": \"\\\"//\\\"\", \"emergency_observed\": false}\r\n");
  EXPECT_FALSE(EmergencyRecord(path).value());
}

TEST_F(RecordTest, InvalidOrUnreadableRecordFailsClosedAndCanBeRepaired) {
  for (const std::string& contents : {"", "not-json", "[]", "{}", "{\"emergency_observed\": 1}",
       "{\"emergency_observed\": \"false\"}", "{\"emergency_observed\": false} trailing",
       "/*comment*/ {\"emergency_observed\": false}", "{\"emergency_observed\": false} //comment",
       "{\"emergency_observed\": false} trailing }", "{\"emergency_observed\": false} /*comment*/ }",
       "{/*comment*/ \"emergency_observed\": false}", "{\"emergency_observed\": false}, {}"}) {
    write(contents);
    EmergencyRecord record(path);
    EXPECT_TRUE(record.value()) << contents;
    std::string error;
    ASSERT_TRUE(record.flush(error)) << error;
    EXPECT_TRUE(EmergencyRecord(path).value());
    ASSERT_TRUE(record.clear(record.revision(), error)) << error;
    EXPECT_FALSE(EmergencyRecord(path).value());
  }
  ASSERT_EQ(0, ::unlink(path.c_str()));
  ASSERT_EQ(0, ::mkdir(path.c_str(), 0700));
  EXPECT_TRUE(EmergencyRecord(path).value());
}

TEST_F(RecordTest, StaleClearCannotEraseAConcurrentObservation) {
  EmergencyRecord record(path);
  record.observe();
  const uint64_t expected = record.revision();
  std::thread observer([&] { record.observe(); });
  observer.join();
  std::string error;
  EXPECT_FALSE(record.clear(expected, error));
  EXPECT_TRUE(record.value());
  ASSERT_TRUE(record.flush(error)) << error;
  EXPECT_TRUE(EmergencyRecord(path).value());
}

TEST_F(RecordTest, PersistenceFailureNeverSuccessfullyClearsAndRestoresMemoryLatch) {
  write("a regular file", directory + "/blocked");
  EmergencyRecord record(directory + "/blocked/record.json");
  EXPECT_TRUE(record.value());
  std::string error;
  EXPECT_FALSE(record.clear(record.revision(), error));
  EXPECT_FALSE(error.empty());
  EXPECT_TRUE(record.value());
  EXPECT_FALSE(record.flush(error));
  EXPECT_TRUE(record.value());
}

TEST_F(RecordTest, BlockedFileIoDoesNotHoldStateLockOrDelayNewEmergency) {
  EmergencyRecord record(path);
  record.observe();
  std::string error;
  ASSERT_TRUE(record.flush(error)) << error;
  ASSERT_EQ(0, ::mkfifo((path + ".tmp").c_str(), 0600));
  const uint64_t expected = record.revision();
  auto clearing = std::async(std::launch::async, [&] {
    std::string clear_error;
    return record.clear(expected, clear_error);
  });
  // No reader exists yet, so the persistence thread blocks in opening the FIFO.
  // observe() must still acquire its short state lock and complete immediately.
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (record.revision() == expected && std::chrono::steady_clock::now() < deadline)
    std::this_thread::yield();
  auto observing = std::async(std::launch::async, [&] { record.observe(); return record.value(); });
  const auto observed = observing.wait_for(std::chrono::milliseconds(100));
  // Always unblock the writer before assertions, including failure paths.
  const int reader = ::open((path + ".tmp").c_str(), O_RDONLY | O_NONBLOCK);
  EXPECT_EQ(std::future_status::ready, observed);
  ASSERT_GE(reader, 0);
  EXPECT_TRUE(observing.get());
  EXPECT_FALSE(clearing.get());  // fsync on a FIFO fails and must relatch.
  ::close(reader);
  EXPECT_TRUE(record.value());
  // The failed atomic write removes the temporary FIFO; the next flush repairs storage.
  ASSERT_TRUE(record.flush(error)) << error;
  EXPECT_TRUE(EmergencyRecord(path).value());
}

TEST_F(RecordTest, RecursiveParentsAndHomeExpansionWork) {
  EmergencyRecord record(directory + "/nested/second/record.json");
  record.observe();
  std::string error;
  ASSERT_TRUE(record.flush(error)) << error;
  EXPECT_TRUE(EmergencyRecord(directory + "/nested/second/record.json").value());
  const char* home = ::getenv("HOME");
  ASSERT_NE(nullptr, home);
  EXPECT_EQ(std::string(home) + "/.ros/emergency.json", expandUserPath("~/.ros/emergency.json"));
  EXPECT_EQ('/', expandUserPath("relative.json")[0]);
}

TEST_F(RecordTest, ManualLimitUpdatePreservesOtherLinesIndentAndCrLf) {
  const std::string config = directory + "/chassis.yaml";
  write("# retain this comment\r\nport: /dev/ttyS0\r\n  manual_drive_max_wheel_speed: 0.2\r\nother: true\r\n", config);
  std::string error;
  ASSERT_TRUE(persistManualSpeedLimit(config, 0.123456789, error)) << error;
  EXPECT_EQ("# retain this comment\r\nport: /dev/ttyS0\r\n  manual_drive_max_wheel_speed: 0.123457\r\nother: true\r\n", read(config));
  write("port: /dev/ttyS0", config);
  ASSERT_TRUE(persistManualSpeedLimit(config, 0.0, error)) << error;
  EXPECT_EQ("port: /dev/ttyS0\nmanual_drive_max_wheel_speed: 0\n", read(config));
  EXPECT_FALSE(persistManualSpeedLimit("", 0.2, error));
  EXPECT_FALSE(persistManualSpeedLimit(config + ".missing", 0.2, error));
  EXPECT_FALSE(persistManualSpeedLimit(config, std::numeric_limits<double>::infinity(), error));
  EXPECT_FALSE(persistManualSpeedLimit(config, -0.2, error));
  EXPECT_EQ(-1, ::access((config + ".tmp").c_str(), F_OK));
}
}  // namespace
}  // namespace grinder_chassis_driver
