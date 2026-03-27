import csv
import time
import logging
from typing import List
from multiprocessing import Event, Process, Queue
from queue import Full, Empty

class CsvLogger:
    def __init__(self, filename: str, header: List[str]):
        self.queue = Queue(maxsize=20000)
        self.filename = filename
        self.header = header
        
        self.stop_event = Event()
        self.thread = Process(target=self._writer_loop, daemon=True)
        self.thread.start()

    def log(self, row):
        try:
            self.queue.put_nowait(row)
        except Full:
            logging.warning("CSV queue full, dropping row")

    def _writer_loop(self):
        file = open(self.filename, "w", newline="")
        writer = csv.writer(file)
        writer.writerow(self.header)
        file.flush()
        last_flush = time.time()

        while not self.stop_event.is_set() or not self.queue.empty():
            batch = []

            try:
                # wait for first row
                batch.append(self.queue.get(timeout=0.5))
            except Empty:
                continue

            # grab more rows without blocking
            while not self.queue.empty() and len(batch) < 200:
                batch.append(self.queue.get_nowait())

            try:
                writer.writerows(batch)
            except Exception as e:
                logging.warning(f"CSV write failed: {e}")

            # flush every second
            if time.time() - last_flush > 1.0:
                file.flush()
                last_flush = time.time()

        file.flush()
        file.close()

    def close(self):
        self.stop_event.set()
        self.thread.join()