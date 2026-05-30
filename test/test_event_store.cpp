#include <filesystem>
#include <limits>
#include <string>

#include <gtest/gtest.h>
#include <sqlite3.h>

#include "ev_ads_runtime_cpp/event_store.hpp"

using namespace ev_ads_runtime_cpp;

namespace {

int count_sqlite_rows(const std::filesystem::path& path) {
  sqlite3* db = nullptr;
  int count = -1;
  if (sqlite3_open(path.string().c_str(), &db) != SQLITE_OK) {
    ADD_FAILURE() << "打开 SQLite 失败: " << path;
    if (db != nullptr) {
      sqlite3_close(db);
    }
    return count;
  }
  sqlite3_stmt* stmt = nullptr;
  if (sqlite3_prepare_v2(db, "SELECT COUNT(*) FROM events;", -1, &stmt, nullptr) != SQLITE_OK) {
    ADD_FAILURE() << "查询 events 表失败: " << sqlite3_errmsg(db);
    sqlite3_close(db);
    return count;
  }
  if (sqlite3_step(stmt) == SQLITE_ROW) {
    count = sqlite3_column_int(stmt, 0);
  }
  sqlite3_finalize(stmt);
  sqlite3_close(db);
  return count;
}

TEST(EventStore, WritesSqliteRowsInBatches) {
  const auto path = std::filesystem::temp_directory_path() / "ev_ads_event_store_test.sqlite";
  std::filesystem::remove(path);
  std::filesystem::remove(path.string() + "-wal");
  std::filesystem::remove(path.string() + "-shm");

  EventStore store;
  EventStoreConfig config;
  config.backend = "sqlite";
  config.path = path.string();
  config.flush_every_n = 8;
  std::string error;
  ASSERT_TRUE(store.open(config, &error)) << error;
  for (int i = 0; i < 25; ++i) {
    EventRecord record;
    record.timestamp_s = 1000.0 + i;
    record.type = "risk";
    record.payload_json = "{\"level\":2,\"score\":0.6}";
    ASSERT_TRUE(store.write(record, &error)) << error;
  }
  ASSERT_TRUE(store.flush(&error)) << error;
  store.close();
  EXPECT_EQ(count_sqlite_rows(path), 25);
}

TEST(EventStoreJson, EscapesArraysAndInvalidNumbers) {
  EXPECT_EQ(escape_json("a\"b\\c"), "a\\\"b\\\\c");
  EXPECT_EQ(string_array_json({"a", "b"}), "[\"a\",\"b\"]");
  EXPECT_EQ(number_json(std::numeric_limits<double>::infinity()), "null");
}

}  // namespace
