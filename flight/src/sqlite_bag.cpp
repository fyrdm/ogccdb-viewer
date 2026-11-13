// SQLite storage (.db3, the rosbag2 default before ROS 2 Iron): tables
// topics(id, name, type, serialization_format[, offered_qos_profiles, ...]),
// messages(id, topic_id, timestamp, data) and, in newer versions,
// message_definitions(topic_type, encoding, encoded_message_definition, ...).
#include <map>
#include <stdexcept>
#include <utility>

#include <sqlite3.h>

#include "ogccdb/flight/bag.hpp"
#include "storage.hpp"

namespace ogccdb::flight {
namespace {

struct Db {
    sqlite3* db = nullptr;
    explicit Db(const std::filesystem::path& file) {
        if (sqlite3_open_v2(file.string().c_str(), &db, SQLITE_OPEN_READONLY, nullptr) != SQLITE_OK) {
            const std::string msg = db ? sqlite3_errmsg(db) : "out of memory";
            sqlite3_close(db);
            throw std::runtime_error("cannot open " + file.string() + ": " + msg);
        }
    }
    ~Db() { sqlite3_close(db); }
};

struct Stmt {
    sqlite3_stmt* s = nullptr;
    Stmt(sqlite3* db, const char* sql) {
        if (sqlite3_prepare_v2(db, sql, -1, &s, nullptr) != SQLITE_OK) {
            throw std::runtime_error(std::string("SQLite bag: ") + sqlite3_errmsg(db));
        }
    }
    ~Stmt() { sqlite3_finalize(s); }
};

std::string text(sqlite3_stmt* s, int col) {
    const unsigned char* t = sqlite3_column_text(s, col);
    return t ? reinterpret_cast<const char*>(t) : "";
}

class SqliteBag : public Bag {
public:
    SqliteBag(std::vector<std::filesystem::path> files, std::filesystem::path path) : files_(std::move(files)) {
        path_ = std::move(path);
        bool first = true;
        for (const auto& f : files_) {
            Db db(f);
            Stmt q(db.db,
                   "SELECT t.name, t.type, t.serialization_format, COUNT(m.id), MIN(m.timestamp), MAX(m.timestamp) "
                   "FROM topics t LEFT JOIN messages m ON m.topic_id = t.id GROUP BY t.id");
            while (sqlite3_step(q.s) == SQLITE_ROW) {
                const std::size_t i = topic_index(text(q.s, 0), text(q.s, 1), text(q.s, 2));
                const auto n = static_cast<std::uint64_t>(sqlite3_column_int64(q.s, 3));
                topics_[i].count += n;
                if (n == 0) continue;
                const std::int64_t t0 = sqlite3_column_int64(q.s, 4), t1 = sqlite3_column_int64(q.s, 5);
                if (first || t0 < start_ns_) start_ns_ = t0;
                if (first || t1 > end_ns_) end_ns_ = t1;
                first = false;
            }
            read_optional_metadata(db);
        }
    }

    std::string storage() const override { return "SQLite"; }

    void read(const std::function<void(const BagMessage&)>& fn,
              const std::function<bool(const BagTopic&)>& want) override {
        for (const auto& f : files_) {
            Db db(f);
            std::map<std::int64_t, std::size_t> ids;
            {
                Stmt q(db.db, "SELECT id, name FROM topics");
                while (sqlite3_step(q.s) == SQLITE_ROW) {
                    const std::size_t i = topic_index(text(q.s, 1), "", "");
                    if (!want || want(topics_[i])) ids[sqlite3_column_int64(q.s, 0)] = i;
                }
            }
            Stmt q(db.db, "SELECT topic_id, timestamp, data FROM messages ORDER BY timestamp");
            while (sqlite3_step(q.s) == SQLITE_ROW) {
                const auto it = ids.find(sqlite3_column_int64(q.s, 0));
                if (it == ids.end()) continue;
                const auto* d = static_cast<const std::uint8_t*>(sqlite3_column_blob(q.s, 2));
                const std::int64_t t = sqlite3_column_int64(q.s, 1);
                fn(BagMessage{&topics_[it->second], t, d, static_cast<std::size_t>(sqlite3_column_bytes(q.s, 2)), t});
            }
        }
    }

private:
    /// QoS profiles and message definitions, which only newer rosbag2 versions store.
    void read_optional_metadata(Db& db) {
        try {
            Stmt q(db.db, "SELECT name, offered_qos_profiles FROM topics");
            while (sqlite3_step(q.s) == SQLITE_ROW) {
                const std::string qos = text(q.s, 1);
                BagTopic& t = topics_[topic_index(text(q.s, 0), "", "")];
                if (!qos.empty() && t.metadata.empty()) t.metadata.emplace_back("offered_qos_profiles", qos);
            }
        } catch (const std::runtime_error&) {
            // no such column
        }
        try {
            Stmt q(db.db, "SELECT topic_type, encoding, encoded_message_definition FROM message_definitions");
            while (sqlite3_step(q.s) == SQLITE_ROW) {
                for (BagTopic& t : topics_) {
                    if (t.type == text(q.s, 0) && t.schema.empty()) {
                        t.schema_encoding = text(q.s, 1);
                        t.schema = text(q.s, 2);
                    }
                }
            }
        } catch (const std::runtime_error&) {
            // no such table
        }
    }

    std::size_t topic_index(const std::string& name, const std::string& type, const std::string& format) {
        for (std::size_t i = 0; i < topics_.size(); ++i) {
            if (topics_[i].name == name) return i;
        }
        BagTopic t;
        t.name = name;
        t.type = type;
        t.encoding = format;
        topics_.push_back(std::move(t));
        return topics_.size() - 1;
    }

    std::vector<std::filesystem::path> files_;
};

}  // namespace

std::unique_ptr<Bag> open_sqlite(const std::vector<std::filesystem::path>& files, const std::filesystem::path& path) {
    return std::make_unique<SqliteBag>(files, path);
}

}  // namespace ogccdb::flight
