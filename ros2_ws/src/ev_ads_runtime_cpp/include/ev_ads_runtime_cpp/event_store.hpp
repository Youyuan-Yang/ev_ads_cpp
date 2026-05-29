#pragma once

#include <fstream>
#include <string>
#include <vector>

#include <sqlite3.h>

namespace ev_ads_runtime_cpp {

struct EventStoreConfig {
  std::string backend = "sqlite";
  std::string path = "/tmp/ev_ads/events.sqlite";
  int flush_every_n = 32;
};

struct EventRecord {
  double timestamp_s = 0.0;
  std::string type;
  std::string payload_json;
};

class EventStore {
 public:
  EventStore() = default;
  ~EventStore();

  EventStore(const EventStore&) = delete;
  EventStore& operator=(const EventStore&) = delete;

  bool open(const EventStoreConfig& config, std::string* error = nullptr);
  bool write(const EventRecord& record, std::string* error = nullptr);
  bool flush(std::string* error = nullptr);
  void close();

  bool opened() const { return opened_; }
  const EventStoreConfig& config() const { return config_; }

 private:
  bool open_jsonl(std::string* error);
  bool open_sqlite(std::string* error);
  bool write_jsonl(const EventRecord& record, std::string* error);
  bool write_sqlite(const EventRecord& record, std::string* error);
  bool begin_sqlite_batch(std::string* error);
  bool commit_sqlite_batch(std::string* error);
  bool exec_sqlite(const char* sql, std::string* error);

  EventStoreConfig config_;
  bool opened_ = false;
  bool sqlite_transaction_open_ = false;
  int pending_writes_ = 0;
  std::ofstream jsonl_;
  sqlite3* db_ = nullptr;
  sqlite3_stmt* insert_stmt_ = nullptr;
};

std::string escape_json(const std::string& text);
std::string string_array_json(const std::vector<std::string>& items);
std::string number_json(double value);

}  // namespace ev_ads_runtime_cpp
