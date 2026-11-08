"""Durable offline job lifecycle. Claiming records a lease; it never starts a worker."""
import json, secrets, sqlite3, time
from .gates import manifest
class Jobs:
    def __init__(self,path):
        self.db=sqlite3.connect(path,timeout=10)
        self.db.execute("CREATE TABLE IF NOT EXISTS jobs (id TEXT PRIMARY KEY, payload TEXT, author TEXT, reviewer TEXT, state TEXT, token TEXT, expires REAL, attempts INTEGER DEFAULT 0, evidence TEXT)")
    def submit(self,payload,author):
        if not author: raise ValueError('Author required')
        key=manifest(payload)
        with self.db: self.db.execute("INSERT OR IGNORE INTO jobs(id,payload,author,state) VALUES(?,?,?,'pending_review')",(key,json.dumps(payload,sort_keys=True),author))
        return key
    def approve(self,key,reviewer):
        if not reviewer: raise ValueError('Reviewer required')
        with self.db:
            c=self.db.execute("UPDATE jobs SET reviewer=?,state='approved' WHERE id=? AND state='pending_review' AND author<>?",(reviewer,key,reviewer))
            if c.rowcount!=1: raise ValueError('Independent approval required')
    def reviewed_manifest(self,key):
        row=self.db.execute("SELECT payload,author,reviewer FROM jobs WHERE id=? AND state='approved'",(key,)).fetchone()
        if row is None:
            raise ValueError('Approved job required')
        payload,author,reviewer=row
        payload=json.loads(payload)
        if manifest(payload)!=key:
            raise ValueError('Job manifest digest mismatch')
        return payload,author,reviewer
    def claim_offline(self,key,now=None,seconds=60):
        if not 0<seconds<=180: raise ValueError('Invalid lease duration')
        now=time.time() if now is None else now
        token=secrets.token_hex(16)
        with self.db:
            c=self.db.execute("UPDATE jobs SET state='leased',token=?,expires=?,attempts=attempts+1 WHERE id=? AND state='approved' AND attempts<2",(token,now+seconds,key))
            if c.rowcount!=1: raise ValueError('Job unavailable')
        return token
    def lease_active(self,key,token,now=None):
        now=time.time() if now is None else now
        return self.db.execute("SELECT 1 FROM jobs WHERE id=? AND token=? AND state='leased' AND expires>?",
                               (key,token,now)).fetchone() is not None
    def finish(self,key,token,evidence,now=None):
        if not isinstance(evidence,str) or len(evidence)!=64 or any(c not in '0123456789abcdef' for c in evidence): raise ValueError('Evidence SHA256 required')
        now=time.time() if now is None else now
        with self.db:
            c=self.db.execute("UPDATE jobs SET state='offline_complete',evidence=?,token=NULL WHERE id=? AND token=? AND state='leased' AND expires>?",(evidence,key,token,now))
            if c.rowcount!=1: raise ValueError('Expired or cancelled lease')
    def fail(self,key,token,now=None):
        now=time.time() if now is None else now
        with self.db:
            self.db.execute("UPDATE jobs SET state='failed',token=NULL WHERE id=? AND token=? AND state='leased' AND expires>?",
                            (key,token,now))
    def recover(self,now=None):
        now=time.time() if now is None else now
        with self.db: self.db.execute("UPDATE jobs SET state=CASE WHEN attempts<2 THEN 'approved' ELSE 'failed' END,token=NULL WHERE state='leased' AND expires<=?",(now,))
    def cancel(self,key):
        with self.db: self.db.execute("UPDATE jobs SET state='cancelled',token=NULL WHERE id=? AND state NOT IN ('offline_complete','failed')",(key,))
    def close(self): self.db.close()
