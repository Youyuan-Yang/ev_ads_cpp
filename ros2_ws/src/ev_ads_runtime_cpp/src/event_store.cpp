#include "ev_ads_runtime_cpp/event_store.hpp"

#include <cmath>
#include <filesystem>
#include <sstream>

namespace ev_ads_runtime_cpp {
namespace {

bool is_sqlite_backend(const std::string& backend) {
  return backend == "sqlite" || backend == "sqlite3" || backend == "db";
}

bool is_jsonl_backend(const std::string& backend) {
  return backend == "jsonl" || backend == "file";
}

void set_error(std::string* error, const std::string& message) {
  if (error != nullptr) {
    *error = message;
  }
}

}  // namespace

EventStore::~EventStore() {
  close();
}

bool EventStore::open(const EventStoreConfig& config, std::string* error) {
  close();
  config_ = config;
  if (config_.flush_every_n <= 0) {
    config_.flush_every_n = 1;
  }

  const std::filesystem::path path(config_.path);
  if (path.has_parent_path()) {
    std::error_code ec;
    std::filesystem::create_directories(path.parent_path(), ec);
    if (ec) {
      set_error(error, "创建事件目录失败: " + ec.message());
      return false;
    }
  }

  if (is_sqlite_backend(config_.backend)) {
    opened_ = open_sqlite(error);
  } else if (is_jsonl_backend(config_.backend)) {
    opened_ = open_jsonl(error);
  } else {
    set_error(error, "未知事件存储后端: " + config_.backend);
    opened_ = false;
  }
  return opened_;
}

bool EventStore::open_jsonl(std::string* error) {
  jsonl_.open(config_.path, std::ios::app);
  if (!jsonl_.is_open()) {
    set_error(error, "打开 JSONL 事件文件失败: " + config_.path);
    return false;
  }
  return true;
}

bool EventStore::open_sqlite(std::string* error) {
  if (sqlite3_open(config_.path.c_str(), &db_) != SQLITE_OK) {
    set_error(error, db_ == nullptr ? "打开 SQLite 失败" : sqlite3_errmsg(db_));
    close();
    return false;
  }
  if (!exec_sqlite("PRAGMA journal_mode=WAL;", error)) {
    close();
    return false;
  }
  if (!exec_sqlite("PRAGMA synchronous=NORMAL;", error)) {
    close();
    return false;
  }
  if (!exec_sqlite(
          "CREATE TABLE IF NOT EXISTS events ("
          "id INTEGER PRIMARY KEY AUTOINCREMENT,"
          "t REAL NOT NULL,"
          "type TEXT NOT NULL,"
          "payload TEXT NOT NULL);",
          error)) {
    close();
    return false;
  }
  if (!exec_sqlite("CREATE INDEX IF NOT EXISTS idx_events_t ON events(t);", error)) {
    close();
    return false;
  }
  const char* sql = "INSERT INTO events(t, type, payload) VALUES(?, ?, ?);";
  if (sqlite3_prepare_v2(db_, sql, -1, &insert_stmt_, nullptr) != SQLITE_OK) {
    set_error(error, sqlite3_errmsg(db_));
    close();
    return false;
  }
  return begin_sqlite_batch(error);
}

bool EventStore::write(const EventRecord& record, std::string* error) {
  if (!opened_) {
    set_error(error, "事件存储未打开");
    return false;
  }
  if (is_sqlite_backend(config_.backend)) {
    return write_sqlite(record, error);
  }
  return write_jsonl(record, error);
}

bool EventStore::write_jsonl(const EventRecord& record, std::string* error) {
  if (!jsonl_.is_open()) {
    set_error(error, "JSONL 事件文件未打开");
    return false;
  }
  jsonl_ << "{\"t\":" << record.timestamp_s
         << ",\"type\":\"" << escape_json(record.type)
         << "\",\"payload\":" << record.payload_json << "}\n";
  ++pending_writes_;
  if (pending_writes_ >= config_.flush_every_n) {
    return flush(error);
  }
  return true;
}

bool EventStore::write_sqlite(const EventRecord& record, std::string* error) {
  if (db_ == nullptr || insert_stmt_ == nullptr) {
    set_error(error, "SQLite 事件库未打开");
    return false;
  }
  sqlite3_bind_double(insert_stmt_, 1, record.timestamp_s);
  sqlite3_bind_text(insert_stmt_, 2, record.type.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_text(insert_stmt_, 3, record.payload_json.c_str(), -1, SQLITE_TRANSIENT);
  const int rc = sqlite3_step(insert_stmt_);
  sqlite3_reset(insert_stmt_);
  sqlite3_clear_bindings(insert_stmt_);
  if (rc != SQLITE_DONE) {
    set_error(error, sqlite3_errmsg(db_));
    return false;
  }
  ++pending_writes_;
  if (pending_writes_ >= config_.flush_every_n) {
    return flush(error);
  }
  return true;
}

bool EventStore::flush(std::string* error) {
  if (!opened_) {
    return true;
  }
  if (is_sqlite_backend(config_.backend)) {
    if (pending_writes_ <= 0) {
      return true;
    }
    if (!commit_sqlite_batch(error)) {
      return false;
    }
    pending_writes_ = 0;
    return begin_sqlite_batch(error);
  }
  if (jsonl_.is_open()) {
    jsonl_.flush();
  }
  pending_writes_ = 0;
  return true;
}

void EventStore::close() {
  if (opened_ && is_sqlite_backend(config_.backend) && sqlite_transaction_open_) {
    std::string ignored;
    commit_sqlite_batch(&ignored);
  } else if (opened_ && jsonl_.is_open()) {
    jsonl_.flush();
  }
  if (jsonl_.is_open()) {
    jsonl_.close();
  }
  if (insert_stmt_ != nullptr) {
    sqlite3_finalize(insert_stmt_);
    insert_stmt_ = nullptr;
  }
  if (db_ != nullptr) {
    sqlite3_close(db_);
    db_ = nullptr;
  }
  opened_ = false;
  sqlite_transaction_open_ = false;
  pending_writes_ = 0;
}

bool EventStore::begin_sqlite_batch(std::string* error) {
  if (sqlite_transaction_open_) {
    return true;
  }
  if (!exec_sqlite("BEGIN IMMEDIATE TRANSACTION;", error)) {
    return false;
  }
  sqlite_transaction_open_ = true;
  return true;
}

bool EventStore::commit_sqlite_batch(std::string* error) {
  if (!sqlite_transaction_open_) {
    return true;
  }
  if (!exec_sqlite("COMMIT;", error)) {
    return false;
  }
  sqlite_transaction_open_ = false;
  return true;
}

bool EventStore::exec_sqlite(const char* sql, std::string* error) {
  char* sqlite_error = nullptr;
  const int rc = sqlite3_exec(db_, sql, nullptr, nullptr, &sqlite_error);
  if (rc != SQLITE_OK) {
    const std::string message = sqlite_error == nullptr ? sqlite3_errmsg(db_) : sqlite_error;
    sqlite3_free(sqlite_error);
    set_error(error, message);
    return false;
  }
  return true;
}

std::string escape_json(const std::string& text) {
  std::ostringstream out;
  for (const char ch : text) {
    switch (ch) {
      case '\\':
        out << "\\\\";
        break;
      case '"':
        out << "\\\"";
        break;
      case '\n':
        out << "\\n";
        break;
      case '\r':
        out << "\\r";
        break;
      case '\t':
        out << "\\t";
        break;
      default:
        out << ch;
        break;
    }
  }
  return out.str();
}

std::string string_array_json(const std::vector<std::string>& items) {
  std::ostringstream out;
  out << "[";
  for (size_t i = 0; i < items.size(); ++i) {
    if (i != 0) {
      out << ",";
    }
    out << "\"" << escape_json(items[i]) << "\"";
  }
  out << "]";
  return out.str();
}

std::string number_json(double value) {
  if (!std::isfinite(value)) {
    return "null";
  }
  return std::to_string(value);
}

}  // namespace ev_ads_runtime_cpp
