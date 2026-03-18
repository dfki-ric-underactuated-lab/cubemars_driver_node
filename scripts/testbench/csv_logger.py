import csv
import queue
import threading
import time
import logging
from typing import List

class CsvLogger:
    def __init__(self, filename: str, header: List[str]):
        self.queue = queue.Queue(maxsize=20000)
        self.running = True

        self.file = open(filename, "w", newline="")
        self.writer = csv.writer(self.file)
        self.writer.writerow(header)
        self.file.flush()

        self.thread = threading.Thread(target=self._writer_loop, daemon=True)
        self.thread.start()

    def log(self, row):
        try:
            self.queue.put_nowait(row)
        except queue.Full:
            logging.warning("CSV queue full, dropping row")

    def _writer_loop(self):
        last_flush = time.time()

        while self.running or not self.queue.empty():
            batch = []

            try:
                # wait for first row
                batch.append(self.queue.get(timeout=0.5))
            except queue.Empty:
                continue

            # grab more rows without blocking
            while not self.queue.empty() and len(batch) < 200:
                batch.append(self.queue.get_nowait())

            try:
                self.writer.writerows(batch)
            except Exception as e:
                logging.warning(f"CSV write failed: {e}")

            # flush every second
            if time.time() - last_flush > 1.0:
                self.file.flush()
                last_flush = time.time()

        self.file.flush()
        self.file.close()

    def close(self):
        self.running = False
        self.thread.join()