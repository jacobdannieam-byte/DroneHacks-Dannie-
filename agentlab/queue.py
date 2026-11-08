import json, sqlite3, time
from .gates import manifest, repair
class Queue:
    def __init__(self,path):
        self.db=sqlite3.connect(path,timeout=10)
        self.db.execute('PRAGMA journal_mode=WAL')
        self.db.execute('CREATE TABLE IF NOT EXISTS proposals (id TEXT PRIMARY KEY, kind TEXT NOT NULL, payload TEXT NOT NULL, status TEXT NOT NULL, created REAL NOT NULL)')
    def add(self,payload,kind='experiment'):
        if kind not in ('experiment','repair'): raise ValueError('Unknown ticket kind')
        key=(repair if kind=='repair' else manifest)(payload)
        with self.db:
            self.db.execute('INSERT OR IGNORE INTO proposals VALUES (?,?,?,?,?)',(key,kind,json.dumps(payload,sort_keys=True),'pending_review',time.time()))
        return key
    def list(self):
        return [dict(id=a,kind=b,payload=json.loads(c),status=d) for a,b,c,d in self.db.execute('SELECT id,kind,payload,status FROM proposals ORDER BY created,id')]
    def reject(self,key):
        with self.db:
            cursor=self.db.execute("UPDATE proposals SET status='rejected' WHERE id=? AND status='pending_review'",(key,))
            if cursor.rowcount!=1: raise ValueError('No pending ticket')
    def close(self): self.db.close()
