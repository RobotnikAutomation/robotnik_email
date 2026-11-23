import os
import tempfile
import unittest
from robotnik_email.email_outbox import EmailOutbox


class OutboxTest(unittest.TestCase):
    def test_persistence_fifo_retry_limits_and_cleanup(self):
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, 'outbox.db')
            q = EmailOutbox(path, 2, 2, 20, 3, 10)
            one = q.enqueue('one', 'fallback', 100)
            two = q.enqueue('two', 'fallback', 101)
            self.assertIsNone(q.enqueue('overflow', '', 102))
            self.assertEqual(q.next(102)[0], one)
            q.record(one, False, 'offline', 102)
            self.assertIsNone(q.next(111))
            q.close()
            q = EmailOutbox(path, 2, 2, 20, 3, 10)
            self.assertEqual(q.next(112)[0], one)
            q.record(one, True, now=112)
            self.assertIsNone(q.next(131))
            self.assertEqual(q.next(132)[0], two)
            q.record(two, True, now=132)
            self.assertEqual(q.count(), 0)
            self.assertEqual(q.db.execute('SELECT status, attempts FROM emails ORDER BY id').fetchall(), [('sent', 2), ('sent', 1)])
            q.cleanup(132 + 30 * 86400 + 1)
            self.assertEqual(q.db.execute('SELECT count(*) FROM emails').fetchone()[0], 0)
            q.close()

    def test_pending_retried_until_expiry(self):
        with tempfile.TemporaryDirectory() as d:
            q = EmailOutbox(os.path.join(d, 'outbox.db'), 10, 2, 20, 3, 60)
            item = q.enqueue('pending', 'fallback', 100)
            q.record(item, False, 'network unavailable', 101)
            before_expiry = 100 + 30 * 86400 - 1
            q.cleanup(before_expiry)
            self.assertEqual(q.next(before_expiry)[0], item)
            q.record(item, False, 'still offline', before_expiry)
            q.cleanup(100 + 30 * 86400)
            self.assertEqual(q.count(), 0)
            self.assertIsNone(q.next(100 + 30 * 86400 + 100))
            q.close()


if __name__ == '__main__':
    unittest.main()
