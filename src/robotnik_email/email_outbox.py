"""Persistent FIFO SMTP outbox. Times are UTC Unix timestamps."""
import os
import sqlite3
import threading
import time


class EmailOutbox:
    def __init__(self, path, max_pending, batch_size, batch_interval, email_interval, retry_interval):
        path = os.path.expanduser(path)
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        self.db = sqlite3.connect(path, timeout=30, check_same_thread=False)
        self.lock = threading.RLock()
        self.max_pending = max_pending
        self.batch_size = batch_size
        self.batch_interval = batch_interval
        self.email_interval = email_interval
        self.retry_interval = retry_interval
        with self.db:
            self.db.execute('PRAGMA journal_mode=WAL')
            self.db.execute('''CREATE TABLE IF NOT EXISTS emails (
                id INTEGER PRIMARY KEY, created REAL NOT NULL, status TEXT NOT NULL,
                message TEXT NOT NULL, fallback TEXT NOT NULL, attempts INTEGER NOT NULL DEFAULT 0,
                last_attempt REAL, next_attempt REAL NOT NULL DEFAULT 0, sent_at REAL,
                last_error TEXT)''')
            self.db.execute('CREATE TABLE IF NOT EXISTS metadata (key TEXT PRIMARY KEY, value REAL NOT NULL)')

    def enqueue(self, message, fallback, now=None):
        now = time.time() if now is None else now
        with self.lock, self.db:
            count = self.db.execute("SELECT count(*) FROM emails WHERE status='pending'").fetchone()[0]
            if count >= self.max_pending:
                return None
            cursor = self.db.execute("INSERT INTO emails(created,status,message,fallback) VALUES (?, 'pending', ?, ?)", (now, message, fallback))
            return cursor.lastrowid

    def count(self):
        with self.lock:
            return self.db.execute("SELECT count(*) FROM emails WHERE status='pending'").fetchone()[0]

    def cleanup(self, now=None):
        now = time.time() if now is None else now
        with self.lock, self.db:
            self.db.execute("DELETE FROM emails WHERE created <= ?", (now - 30 * 86400,))

    def next(self, now=None):
        now = time.time() if now is None else now
        with self.lock:
            last = self.db.execute("SELECT value FROM metadata WHERE key='last_attempt'").fetchone()
            if last and now < last[0] + self.email_interval:
                return None
            batch = self.db.execute("SELECT value FROM metadata WHERE key='batch_until'").fetchone()
            if batch and now < batch[0]:
                return None
            # Strict FIFO: a failed first item waits for its retry before later items.
            row = self.db.execute("SELECT id,message,fallback,next_attempt FROM emails WHERE status='pending' ORDER BY id LIMIT 1").fetchone()
            return row[:3] if row and row[3] <= now else None

    def record(self, item_id, success, error='', now=None):
        now = time.time() if now is None else now
        with self.lock, self.db:
            self.db.execute("UPDATE emails SET attempts=attempts+1,last_attempt=?,next_attempt=?,status=?,sent_at=?,last_error=? WHERE id=?", (now, now + self.retry_interval, 'sent' if success else 'pending', now if success else None, error or None, item_id))
            count = self.db.execute("SELECT value FROM metadata WHERE key='batch_count'").fetchone()
            count = int(count[0]) + 1 if count else 1
            self.db.execute("INSERT OR REPLACE INTO metadata VALUES ('last_attempt', ?)", (now,))
            if count >= self.batch_size:
                self.db.execute("INSERT OR REPLACE INTO metadata VALUES ('batch_until', ?)", (now + self.batch_interval,))
                count = 0
            self.db.execute("INSERT OR REPLACE INTO metadata VALUES ('batch_count', ?)", (count,))

    def close(self):
        with self.lock:
            self.db.close()
