#include <cassert>
#include <filesystem>
#include <iostream>
#include <limits>
#include <string>

#include <sqlite3.h>

#include "ev_ads_runtime_cpp/event_store.hpp"

using namespace ev_ads_runtime_cpp;

namespace {

int count_sqlite_rows(const std::filesystem::path& path) {
  sqlite3* db = nullptr;
  int count = -1;
  assert(sqlite3_open(path.string().c_str(), &db) == SQLITE_OK);
  sqlite3_stmt* stmt = nullptr;
  assert(sqlite3_prepare_v2(db, "SELECT COUNT(*) FROM events;", -1, &stmt, nullptr) == SQLITE_OK);
  if (sqlite3_step(stmt) == SQLITE_ROW) {
    count = sqlite3_column_int(stmt, 0);
  }
  sqlite3_finalize(stmt);
  sqlite3_close(db);
  return count;
}

void test_sqlite_store() {
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
  assert(store.open(config, &error));
  for (int i = 0; i < 25; ++i) {
    EventRecord record;
    record.timestamp_s = 1000.0 + i;
    record.type = "risk";
    record.payload_json = "{\"level\":2,\"score\":0.6}";
    assert(store.write(record, &error));
  }
  assert(store.flush(&error));
  store.close();
  assert(count_sqlite_rows(path) == 25);
}

void test_json_helpers() {
  assert(escape_json("a\"b\\c") == "a\\\"b\\\\c");
  assert(string_array_json({"a", "b"}) == "[\"a\",\"b\"]");
  assert(number_json(std::numeric_limits<double>::infinity()) == "null");
}

}  // namespace

int main() {
  test_sqlite_store();
  test_json_helpers();
  std::cout << "test_event_store ok\n";
  return 0;
}
